#!/usr/bin/env python3
# coding=UTF-8

import os
import sys
import numpy as np
import timeit
from threading import Lock

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# MAVROS imports
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Import your drone model and optimizer
from .drone_model import DroneModel
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


class DroneMPCController(Node):
    def __init__(self):
        super().__init__('drone_mpc_controller')

        # ============= DECLARE PARAMETERS =============
        self.declare_parameter('control_rate', 50)
        self.declare_parameter('t_horizon', 1.0)
        self.declare_parameter('n_nodes', 20)

        # Weights parameters
        self.declare_parameter('weights.position', [200.0, 200.0, 500.0])
        self.declare_parameter('weights.velocity', [1.0, 1.0, 1.0])
        self.declare_parameter('weights.quaternion', [5.0, 5.0, 200.0, 1.0])
        self.declare_parameter('weights.angular_velocity', [1.0, 1.0, 1.0])
        self.declare_parameter('weights.control', [6.0, 6.0, 6.0, 6.0])

        # Target parameters
        self.declare_parameter('target.x', 0.0)
        self.declare_parameter('target.y', 0.0)
        self.declare_parameter('target.z', 2.0)

        # Drone physical parameters
        self.declare_parameter('mass', 0.5)
        self.declare_parameter('gravity', 9.8)
        self.declare_parameter('inertia.I_xx', 0.00232)
        self.declare_parameter('inertia.I_yy', 0.00232)
        self.declare_parameter('inertia.I_zz', 0.00400)

        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.t_horizon = self.get_parameter('t_horizon').value
        self.n_nodes = self.get_parameter('n_nodes').value

        # Stato del drone (13 stati: p, v, q, w)
        self.current_state = np.zeros(13)
        self.current_state[6] = 1.0  # quaternione iniziale (w=1)
        self.state_lock = Lock()

        # Setpoint desiderato
        target_x = self.get_parameter('target.x').value
        target_y = self.get_parameter('target.y').value
        target_z = self.get_parameter('target.z').value
        self.target_state = np.array([target_x, target_y, target_z, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])

        # Stato MAVROS
        self.mavros_state = State()
        self.connected = False

        # ============= SETUP ACADOS =============
        self.get_logger().info("Inizializzazione ACADOS MPC...")
        self.get_logger().info("ATTENZIONE: Prima compilazione ACADOS può richiedere 1-2 minuti...")
        try:
            self.setup_acados()
            self.get_logger().info("ACADOS inizializzato correttamente!")
        except Exception as e:
            self.get_logger().error(f"Errore inizializzazione ACADOS: {e}")
            raise

        # ============= QoS PROFILES =============
        # State topic uses RELIABLE + TRANSIENT_LOCAL
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Sensor topics use BEST_EFFORT
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Per i comandi usiamo RELIABLE
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ============= ROS 2 SUBSCRIBERS =============
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            state_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            sensor_qos
        )

        # ============= ROS 2 PUBLISHERS =============
        # Pubblichiamo thrust e attitude
        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            command_qos
        )

        # Per debugging
        self.setpoint_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            command_qos
        )

        # ============= ROS 2 SERVICE CLIENTS =============
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Don't wait for services during init - check in control loop instead
        self.get_logger().info("Servizi MAVROS creati (saranno controllati durante l'esecuzione)")

        self.get_logger().info("Drone MPC Controller inizializzato!")

        # ============= TIMERS =============
        # Main control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop_callback
        )

        # Statistics
        self.solve_times = []
        self.iteration_count = 0
        self.offboard_armed = False

    def setup_acados(self):
        """Setup del solver ACADOS MPC"""
        # Crea il modello del drone
        drone_model_obj = DroneModel()
        model = drone_model_obj.model
        constraint = drone_model_obj.constraint

        self.T = self.t_horizon
        self.N = self.n_nodes

        # Ensure current working directory is correct
        # For ROS 2, we'll use the package share directory
        package_share_dir = os.path.join(os.path.expanduser('~'), 'drone', 'drone_mpc_ros2', 'src', 'drone_mpc_control')
        self.acados_models_dir = os.path.join(package_share_dir, 'acados_models')
        if not os.path.exists(self.acados_models_dir):
            os.makedirs(self.acados_models_dir)
        os.chdir(package_share_dir)

        acados_source_path = os.environ.get('ACADOS_SOURCE_DIR',
                                           os.path.expanduser('~/acados'))

        nx = model.x.size()[0]  # 13 stati
        self.nx = nx
        nu = model.u.size()[0]  # 4 controlli (T, Mx, My, Mz)
        self.nu = nu
        ny = nx + nu
        n_params = len(model.p)

        # Get weight parameters
        weights_pos = self.get_parameter('weights.position').value
        weights_vel = self.get_parameter('weights.velocity').value
        weights_quat = self.get_parameter('weights.quaternion').value
        weights_ang_vel = self.get_parameter('weights.angular_velocity').value
        weights_control = self.get_parameter('weights.control').value

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
        Q = np.diag(weights_pos + weights_vel + weights_quat + weights_ang_vel)

        # Pesi per controlli: [T, Mx, My, Mz]
        R = np.diag(weights_control)

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
        json_file = os.path.join(package_share_dir, model.name + '_acados_ocp.json')
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
            self.get_logger().warn(f"ACADOS solver returned status {status}")
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
        att_cmd.header.stamp = self.get_clock().now().to_msg()
        att_cmd.header.frame_id = "base_link"

        # Tipo di controllo: thrust + body rate
        att_cmd.type_mask = AttitudeTarget.IGNORE_ATTITUDE

        # Thrust normalizzato (PX4 si aspetta [0,1])
        # Assumiamo che il tuo modello usi Newton, quindi normalizziamo
        mass = self.get_parameter('mass').value
        g = self.get_parameter('gravity').value
        hover_thrust = mass * g
        thrust_normalized = (thrust + hover_thrust) / (2 * hover_thrust)
        thrust_normalized = np.clip(thrust_normalized, 0.0, 1.0)
        att_cmd.thrust = thrust_normalized

        # Body rates (rad/s) - i momenti vengono convertiti in rate
        # Questo è semplificato, in realtà dovremmo fare l'inversa della dinamica
        I_xx = self.get_parameter('inertia.I_xx').value
        I_yy = self.get_parameter('inertia.I_yy').value
        I_zz = self.get_parameter('inertia.I_zz').value
        I_inv = np.diag([1/I_xx, 1/I_yy, 1/I_zz])

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
        self.get_logger().info(f"Nuovo target: [{x:.2f}, {y:.2f}, {z:.2f}]")

    def control_loop_callback(self):
        """Main control loop - chiamato dal timer"""
        # Se non siamo ancora in OFFBOARD e armati, gestiamo l'attivazione
        if not self.offboard_armed:
            if not self.connected:
                # Log once every 50 calls to avoid spam
                if not hasattr(self, '_wait_connect_count'):
                    self._wait_connect_count = 0
                    self.get_logger().info("Attesa connessione MAVROS...")
                self._wait_connect_count += 1
                if self._wait_connect_count >= 50:
                    self._wait_connect_count = 0
                return
            self.handle_offboard_arming()
            return

        # Risolvi MPC
        u_optimal, solve_time = self.solve_mpc()
        self.solve_times.append(solve_time)

        # Pubblica controllo
        self.publish_control(u_optimal)

        # Stampa statistiche ogni 100 iterazioni
        self.iteration_count += 1
        if self.iteration_count >= 100:
            avg_time = np.mean(self.solve_times)
            max_time = np.max(self.solve_times)
            self.get_logger().info(f"MPC solve time - Avg: {avg_time*1000:.2f}ms, Max: {max_time*1000:.2f}ms")

            with self.state_lock:
                pos_error = np.linalg.norm(
                    self.current_state[:3] - self.target_state[:3]
                )
            self.get_logger().info(f"Errore posizione: {pos_error:.3f}m")
            self.solve_times = []
            self.iteration_count = 0

    def handle_offboard_arming(self):
        """Gestisce l'attivazione di OFFBOARD mode e arming"""
        # Log current state occasionally
        if not hasattr(self, '_arming_log_count'):
            self._arming_log_count = 0
            self.get_logger().info(f"Tentativo arming - Mode: {self.mavros_state.mode}, Armed: {self.mavros_state.armed}")
        self._arming_log_count += 1
        if self._arming_log_count >= 50:
            self._arming_log_count = 0

        # Pubblica setpoint per mantenere la connessione
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.z = 2.0
        self.setpoint_pose_pub.publish(pose)

        # Controlla se dobbiamo cambiare modalità o armare
        if self.mavros_state.mode != "OFFBOARD":
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(req)
            # Non aspettiamo la risposta per non bloccare il loop
        elif not self.mavros_state.armed:
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            # Non aspettiamo la risposta per non bloccare il loop

        # Controlla se siamo pronti
        if self.mavros_state.armed and self.mavros_state.mode == "OFFBOARD":
            self.get_logger().info("Drone pronto! Controllo MPC attivo.")
            self.offboard_armed = True


def main(args=None):
    rclpy.init(args=args)

    try:
        controller = DroneMPCController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Errore: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
