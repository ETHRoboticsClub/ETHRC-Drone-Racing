#!/usr/bin/env python
# coding=UTF-8

import os
import sys
import rospy
import numpy as np
import timeit
from threading import Lock

# MAVROS imports
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Import your drone model and optimizer
from drone_model import DroneModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
import scipy.linalg
from types import SimpleNamespace


def quaternion_to_rotation_matrix(q):
    """Converte quaternione (w,x,y,z) in matrice di rotazione"""
    w, x, y, z = q[0], q[1], q[2], q[3]
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    return R


class DroneMPCController:
    def __init__(self):
        rospy.init_node('drone_mpc_controller', anonymous=True)
        
        # ============= PARAMETRI =============
        self.control_rate = 50  # Hz (deve essere >= 2 Hz per OFFBOARD)
        self.t_horizon = 1.0    # Orizzonte MPC in secondi
        self.n_nodes = 20       # Nodi di predizione
        
        # Stato del drone (13 stati: p, v, q, w)
        self.current_state = np.zeros(13)
        self.current_state[6] = 1.0  # quaternione iniziale (w=1)
        self.state_lock = Lock()
        
        # Setpoint desiderato
        self.target_state = np.array([0, 0, 2, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])
        
        # Stato MAVROS
        self.mavros_state = State()
        self.connected = False
        
        # ============= SETUP ACADOS =============
        rospy.loginfo("Inizializzazione ACADOS MPC...")
        self.setup_acados()
        rospy.loginfo("ACADOS inizializzato correttamente!")
        
        # ============= ROS SUBSCRIBERS =============
        self.state_sub = rospy.Subscriber(
            '/mavros/state', 
            State, 
            self.state_callback,
            queue_size=1
        )
        
        self.odom_sub = rospy.Subscriber(
            '/mavros/local_position/odom',
            Odometry,
            self.odom_callback,
            queue_size=1
        )
        
        self.imu_sub = rospy.Subscriber(
            '/mavros/imu/data',
            Imu,
            self.imu_callback,
            queue_size=1
        )
        
        # ============= ROS PUBLISHERS =============
        # Pubblichiamo thrust e attitude
        self.attitude_pub = rospy.Publisher(
            '/mavros/setpoint_raw/attitude',
            AttitudeTarget,
            queue_size=10
        )
        
        # Per debugging
        self.setpoint_pose_pub = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )
        
        # ============= ROS SERVICES =============
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        rospy.loginfo("Drone MPC Controller inizializzato!")

    def setup_acados(self):
        """Setup del solver ACADOS MPC"""
        # Crea il modello del drone
        drone_model_obj = DroneModel()
        model = drone_model_obj.model
        constraint = drone_model_obj.constraint
        
        self.T = self.t_horizon
        self.N = self.n_nodes
        
        # Ensure current working directory is correct
        script_dir = os.path.dirname(os.path.realpath(__file__))
        os.chdir(script_dir)
        self.acados_models_dir = './acados_models'
        if not os.path.exists(self.acados_models_dir):
            os.makedirs(self.acados_models_dir)
        
        acados_source_path = os.environ.get('ACADOS_SOURCE_DIR', 
                                           os.path.expanduser('~/acados'))
        
        nx = model.x.size()[0]  # 13 stati
        self.nx = nx
        nu = model.u.size()[0]  # 4 controlli (T, Mx, My, Mz)
        self.nu = nu
        ny = nx + nu
        n_params = len(model.p)
        
        # ============= CREATE OCP =============
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ocp.model = model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.T
        
        # Parametri
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)
        
        # ============= COST FUNCTION =============
        # Pesi per stati: [p(3), v(3), q(4), w(3)]
        Q = np.diag([
            200, 200, 500,      # posizione (x,y,z) - peso alto su z
            1, 1, 1,            # velocità
            5, 5, 200,          # quaternione (peso alto su orientamento)
            1, 1, 1             # velocità angolare
        ])
        
        # Pesi per controlli: [T, Mx, My, Mz]
        R = np.diag([6, 6, 6, 6])
        
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = Q
        
        # Vx mappa gli stati agli output del costo
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = np.eye(nx)
        
        # Vu mappa i controlli agli output del costo
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[nx:, :] = np.eye(nu)
        
        # ============= CONSTRAINTS =============
        # Vincoli sui controlli
        ocp.constraints.lbu = np.concatenate((
            np.array([constraint.T_min]), 
            constraint.M_min
        ))
        ocp.constraints.ubu = np.concatenate((
            np.array([constraint.T_max]), 
            constraint.M_max
        ))
        ocp.constraints.idxbu = np.array(range(nu))
        
        # Vincoli sulla velocità angolare
        ocp.constraints.lbx = constraint.w_min
        ocp.constraints.ubx = constraint.w_max
        ocp.constraints.idxbx = np.array([10, 11, 12])  # indici di w nel vettore di stato
        
        # Stato iniziale
        x_init = np.zeros(nx)
        x_init[6] = 1.0  # quaternione w=1
        ocp.constraints.x0 = x_init
        
        # Riferimenti
        x_ref = np.zeros(nx)
        x_ref[6] = 1.0
        u_ref = np.zeros(nu)
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref
        
        # ============= SOLVER OPTIONS =============
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        
        # Compila il solver
        json_file = os.path.join(script_dir, model.name + '_acados_ocp.json')
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        self.integrator = AcadosSimSolver(ocp, json_file=json_file)
        
    def state_callback(self, msg):
        """Callback per lo stato MAVROS"""
        self.mavros_state = msg
        self.connected = msg.connected
        
    def odom_callback(self, msg):
        """Callback per odometria (posizione e velocità)"""
        with self.state_lock:
            # Posizione
            self.current_state[0] = msg.pose.pose.position.x
            self.current_state[1] = msg.pose.pose.position.y
            self.current_state[2] = msg.pose.pose.position.z
            
            # Velocità lineare
            self.current_state[3] = msg.twist.twist.linear.x
            self.current_state[4] = msg.twist.twist.linear.y
            self.current_state[5] = msg.twist.twist.linear.z
            
            # Quaternione (w, x, y, z)
            self.current_state[6] = msg.pose.pose.orientation.w
            self.current_state[7] = msg.pose.pose.orientation.x
            self.current_state[8] = msg.pose.pose.orientation.y
            self.current_state[9] = msg.pose.pose.orientation.z
            
    def imu_callback(self, msg):
        """Callback per IMU (velocità angolare)"""
        with self.state_lock:
            # Velocità angolare (body frame)
            self.current_state[10] = msg.angular_velocity.x
            self.current_state[11] = msg.angular_velocity.y
            self.current_state[12] = msg.angular_velocity.z
            
    def solve_mpc(self):
        """Risolve il problema MPC e restituisce il controllo ottimo"""
        with self.state_lock:
            x_current = self.current_state.copy()
        
        # Imposta lo stato iniziale come vincolo
        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)
        
        # Imposta il riferimento per tutti i nodi
        target_ref = self.target_state.copy()
        u_ref = np.zeros(self.nu)
        y_ref = np.concatenate((target_ref, u_ref))
        
        for i in range(self.N):
            self.solver.set(i, 'yref', y_ref)
        self.solver.set(self.N, 'yref', target_ref)
        
        # Risolvi
        start_time = timeit.default_timer()
        status = self.solver.solve()
        solve_time = timeit.default_timer() - start_time
        
        if status != 0:
            rospy.logwarn(f"ACADOS solver returned status {status}")
            return None, solve_time
        
        # Ottieni il controllo ottimo
        u_optimal = self.solver.get(0, 'u')
        
        return u_optimal, solve_time
        
    def publish_control(self, u):
        """Pubblica i comandi di controllo a MAVROS"""
        if u is None:
            return
            
        # u = [thrust, Mx, My, Mz]
        thrust = u[0]
        moments = u[1:4]
        
        # Crea il messaggio AttitudeTarget
        att_cmd = AttitudeTarget()
        att_cmd.header.stamp = rospy.Time.now()
        att_cmd.header.frame_id = "base_link"
        
        # Tipo di controllo: thrust + body rate
        att_cmd.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        
        # Thrust normalizzato (PX4 si aspetta [0,1])
        # Assumiamo che il tuo modello usi Newton, quindi normalizziamo
        mass = 0.5  # kg (dal tuo modello)
        g = 9.8
        hover_thrust = mass * g
        thrust_normalized = (thrust + hover_thrust) / (2 * hover_thrust)
        thrust_normalized = np.clip(thrust_normalized, 0.0, 1.0)
        att_cmd.thrust = thrust_normalized
        
        # Body rates (rad/s) - i momenti vengono convertiti in rate
        # Questo è semplificato, in realtà dovremmo fare l'inversa della dinamica
        I_inv = np.diag([1/2.32e-3, 1/2.32e-3, 1/4.00e-3])
        with self.state_lock:
            w_current = self.current_state[10:13]
        
        # Rate desiderato basato sui momenti
        # Questo è un'approssimazione semplice
        w_dot = I_inv @ moments
        dt = 1.0 / self.control_rate
        w_desired = w_current + w_dot * dt
        
        att_cmd.body_rate.x = w_desired[0]
        att_cmd.body_rate.y = w_desired[1]
        att_cmd.body_rate.z = w_desired[2]
        
        # Pubblica
        self.attitude_pub.publish(att_cmd)
        
    def set_target_position(self, x, y, z):
        """Imposta una nuova posizione target"""
        self.target_state[0] = x
        self.target_state[1] = y
        self.target_state[2] = z
        rospy.loginfo(f"Nuovo target: [{x:.2f}, {y:.2f}, {z:.2f}]")
        
    def offboard_mode_and_arm(self):
        """Attiva la modalità OFFBOARD e arma il drone"""
        rate = rospy.Rate(self.control_rate)
        
        # Invia setpoint per 2 secondi prima di attivare OFFBOARD
        rospy.loginfo("Invio setpoint iniziali...")
        pose = PoseStamped()
        pose.pose.position.z = 2.0
        
        for _ in range(2 * self.control_rate):
            if rospy.is_shutdown():
                return False
            self.setpoint_pose_pub.publish(pose)
            rate.sleep()
        
        # Attiva OFFBOARD
        rospy.loginfo("Attivazione modalità OFFBOARD...")
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'
        
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        
        last_req = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.mavros_state.mode != "OFFBOARD" and \
               (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD abilitato")
                last_req = rospy.Time.now()
            else:
                if not self.mavros_state.armed and \
                   (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Drone armato")
                    last_req = rospy.Time.now()
            
            if self.mavros_state.armed and self.mavros_state.mode == "OFFBOARD":
                rospy.loginfo("Drone pronto! Controllo MPC attivo.")
                return True
                
            self.setpoint_pose_pub.publish(pose)
            rate.sleep()
            
        return False
        
    def run(self):
        """Loop principale di controllo"""
        rate = rospy.Rate(self.control_rate)
        
        # Aspetta la connessione
        rospy.loginfo("Attesa connessione con PX4...")
        while not rospy.is_shutdown() and not self.connected:
            rate.sleep()
        rospy.loginfo("Connesso a PX4!")
        
        # Attiva OFFBOARD e arma
        if not self.offboard_mode_and_arm():
            rospy.logerr("Impossibile attivare OFFBOARD mode")
            return
        
        # Imposta il target
        self.set_target_position(0, 0, 2)
        
        rospy.loginfo("Inizio controllo MPC...")
        solve_times = []
        
        try:
            while not rospy.is_shutdown():
                # Risolvi MPC
                u_optimal, solve_time = self.solve_mpc()
                solve_times.append(solve_time)
                
                # Pubblica controllo
                self.publish_control(u_optimal)
                
                # Stampa statistiche ogni 100 iterazioni
                if len(solve_times) >= 100:
                    avg_time = np.mean(solve_times)
                    max_time = np.max(solve_times)
                    rospy.loginfo(f"MPC solve time - Avg: {avg_time*1000:.2f}ms, Max: {max_time*1000:.2f}ms")
                    
                    with self.state_lock:
                        pos_error = np.linalg.norm(
                            self.current_state[:3] - self.target_state[:3]
                        )
                    rospy.loginfo(f"Errore posizione: {pos_error:.3f}m")
                    solve_times = []
                
                rate.sleep()
                
        except rospy.ROSInterruptException:
            pass
        except Exception as e:
            rospy.logerr(f"Errore nel loop di controllo: {e}")


if __name__ == '__main__':
    try:
        controller = DroneMPCController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo terminato")