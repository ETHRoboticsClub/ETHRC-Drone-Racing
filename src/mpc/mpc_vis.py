#!/usr/bin/env python3
# coding=UTF-8
"""
mpc_vis.py - simplified threaded matplotlib visualizer for MPC with incremental video saving.
Only a single 3D plot (North, East, Down) is shown.

Usage:
    vis = MPCVisualizer(horizon=20, max_history=200, goal=[N,E,D])
    vis.start()
    vis.update(current_position, x_pred_all, u_pred_all)
    vis.stop()
"""
import threading
import queue
import numpy as np
import time
import warnings
import os

# Try to set a GUI backend before importing pyplot to reduce warnings.
_headless = False
try:
    import matplotlib
    try:
        matplotlib.use('TkAgg')
    except Exception:
        pass
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    if not matplotlib.is_interactive():
        try:
            matplotlib.use('TkAgg')
        except Exception:
            matplotlib.use('Agg')
            _headless = True
            warnings.warn("MPCVisualizer: running in headless mode (Agg backend). Plot window won't show.")
except Exception:
    _headless = True
    warnings.warn("MPCVisualizer: matplotlib not available or failed to initialize. Running headless.")
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

class MPCVisualizer:
    """
    Minimal 3D-only visualizer for MPC predictions and current pose.
    Uses incremental video saving to avoid memory limits.
    """
    def __init__(self, horizon=20, max_history=200, goal=None, draw_every=1, 
                 record_video=False, video_filename='mpc_visualization.mp4', video_fps=20,
                 waypoints=None):
        self.q = queue.Queue()
        self.running = threading.Event()
        self.thread = None
        self.horizon = horizon
        self.max_history = max_history
        self.real_positions = []

        # store goal as numpy array or None
        if goal is not None:
            try:
                g = np.asarray(goal).ravel()
                self.goal = g[:3].astype(float)
            except Exception:
                self.goal = None
        else:
            self.goal = None

        # Store waypoints for visualization
        self.waypoints = waypoints
        self._waypoint_handles = []
        self._waypoint_texts = []

        # internal handles for drawn objects
        self._pred_line = None
        self._real_point = None
        self._trail_line = None
        self._goal_handle = None
        self._goal_text = None
        self._horizon_markers = None
        self._velocity_vector = None
        self._thrust_vector = None

        # drawing throttling
        self._draw_every = max(1, int(draw_every))
        self._draw_count = 0
        
        # video recording with incremental saving
        self.record_video = record_video
        self.video_filename = video_filename
        self.video_fps = video_fps
        self._video_writer = None
        self._frame_count = 0
        
        if record_video:
            try:
                import imageio
                self._imageio_available = True
                print(f"MPCVisualizer: imageio available, will save video to {video_filename}")
            except ImportError:
                self._imageio_available = False
                warnings.warn("MPCVisualizer: imageio not available. Install with: pip install imageio imageio-ffmpeg")
                self.record_video = False

    def start(self):
        """Start plotting thread."""
        if self.thread is None:
            self.running.set()
            if _headless:
                self.thread = threading.Thread(target=self._run_headless, daemon=True)
                self.thread.start()
            else:
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()

    def stop(self, timeout=2.0):
        """Stop plotting thread and close video writer."""
        self.running.clear()
        try:
            self.q.put_nowait(None)
        except Exception:
            pass
        if self.thread is not None:
            self.thread.join(timeout=timeout)
            self.thread = None
        
        # Video writer is closed in the thread itself
        if self._video_writer is not None:
            print(f"MPCVisualizer: Video saved with {self._frame_count} frames")

    def update(self, current_position, x_pred_all=None, u_pred_all=None, goal=None, 
               current_velocity=None, current_thrust_vector=None, current_waypoint_idx=None):
        """Queue data for plotting."""
        if goal is not None:
            try:
                g = np.asarray(goal).ravel()
                self.goal = g[:3].astype(float)
            except Exception as e:
                print("Visualizer: invalid goal passed to update():", e)
        
        try:
            curp = np.asarray(current_position, dtype=float).ravel()[:3].copy()
        except Exception:
            curp = np.zeros(3, dtype=float)
        
        vel = None
        if current_velocity is not None:
            try:
                vel = np.asarray(current_velocity, dtype=float).ravel()[:3].copy()
            except Exception:
                vel = None
        
        thrust = None
        if current_thrust_vector is not None:
            try:
                thrust = np.asarray(current_thrust_vector, dtype=float).ravel()[:3].copy()
            except Exception:
                thrust = None
        
        xp = None if x_pred_all is None else np.asarray(x_pred_all, dtype=float).copy()
        up = None if u_pred_all is None else np.asarray(u_pred_all, dtype=float).copy()
        self.q.put((curp, xp, up, vel, thrust, current_waypoint_idx))

    def _run_headless(self):
        """Simple queue drainer for headless mode."""
        while self.running.is_set():
            try:
                item = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            current_position, x_pred_all, u_pred_all, current_velocity, current_thrust_vector, _ = item
            self.real_positions.append(current_position)
            if len(self.real_positions) > self.max_history:
                self.real_positions.pop(0)

    def _run(self):
        """Thread main: build figure and loop processing queue updates with incremental video saving."""
        try:
            # Initialize video writer if recording
            if self.record_video and self._imageio_available:
                try:
                    import imageio
                    self._video_writer = imageio.get_writer(
                        self.video_filename,
                        fps=self.video_fps,
                        codec='libx264',
                        quality=8,  # 1-10, 10 = best quality
                        pixelformat='yuv420p'
                    )
                    print(f"MPCVisualizer: Video writer initialized for {self.video_filename}")
                except Exception as e:
                    print(f"MPCVisualizer: Failed to initialize video writer: {e}")
                    self._video_writer = None
                    self.record_video = False
            
            plt.ion()
            fig = plt.figure(figsize=(8, 8))
            ax3d = fig.add_subplot(1, 1, 1, projection='3d')
            ax3d.set_title('MPC predicted trajectory (NED)')
            ax3d.set_xlabel('North (m)')
            ax3d.set_ylabel('East (m)')
            ax3d.set_zlabel('Altitude (m)')

            # initialize empty artists
            (self._pred_line,) = ax3d.plot([], [], [], '--', linewidth=2, label='predicted')
            (self._real_point,) = ax3d.plot([], [], [], 'o', color='tab:orange', label='real')
            (self._trail_line,) = ax3d.plot([], [], [], '-', color='tab:gray', alpha=0.6, label='trail')
            
            self._velocity_vector = None
            self._thrust_vector = None
            
            ax3d.legend()
            plt.tight_layout()
            fig.canvas.draw()
            plt.show(block=False)

            # Plot initial goal
            if self.goal is not None:
                try:
                    gx, gy, gz = float(self.goal[0]), float(self.goal[1]), -float(self.goal[2])
                    self._goal_handle = ax3d.scatter([gx], [gy], [gz], marker='*', s=200, c='red', label='Goal')
                    self._goal_text = ax3d.text(gx, gy, gz, 'Goal', color='red', fontsize=9)
                except Exception as e:
                    print("Visualizer: error plotting initial goal:", e)

            # Plot all waypoints
            if self.waypoints is not None and len(self.waypoints) > 0:
                try:
                    for i, waypoint in enumerate(self.waypoints):
                        wp_x, wp_y, wp_z = float(waypoint[0]), float(waypoint[1]), -float(waypoint[2])
                        
                        colors = ['cyan', 'magenta', 'yellow', 'orange', 'lime', 'pink', 'brown', 'gray']
                        color = colors[i % len(colors)]
                        
                        wp_handle = ax3d.scatter([wp_x], [wp_y], [wp_z], 
                                                marker='o', s=150, c=color, 
                                                edgecolors='black', linewidths=2,
                                                alpha=0.8, label=f'WP{i+1}')
                        self._waypoint_handles.append(wp_handle)
                        
                        wp_text = ax3d.text(wp_x, wp_y, wp_z + 0.3,
                                           f'{i+1}', color='black', fontsize=12, 
                                           fontweight='bold', ha='center')
                        self._waypoint_texts.append(wp_text)
                    
                    print(f"Visualizer: plotted {len(self.waypoints)} waypoints")
                except Exception as e:
                    print("Visualizer: error plotting waypoints:", e)

            while self.running.is_set():
                try:
                    item = self.q.get(timeout=0.2)
                except queue.Empty:
                    try:
                        fig.canvas.flush_events()
                    except Exception:
                        pass
                    continue

                if item is None:
                    break

                current_position, x_pred_all, u_pred_all, current_velocity, current_thrust_vector, current_waypoint_idx = item

                # update trail
                self.real_positions.append([current_position[0], current_position[1], -current_position[2]])
                if len(self.real_positions) > self.max_history:
                    self.real_positions.pop(0)
                trail = np.array(self.real_positions)

                # predicted positions
                if x_pred_all is not None and x_pred_all.size != 0:
                    try:
                        p_pred = x_pred_all[:, 0:3].copy()
                        p_pred[:, 2] = -p_pred[:, 2]
                        self._pred_line.set_data(p_pred[:, 0], p_pred[:, 1])
                        self._pred_line.set_3d_properties(p_pred[:, 2])
                        
                        # Horizon markers
                        n_horizon_pts = min(len(p_pred), self.horizon + 1)
                        if n_horizon_pts > 0:
                            if self._horizon_markers is not None:
                                try:
                                    self._horizon_markers.remove()
                                except Exception:
                                    pass
                            
                            colors = plt.cm.viridis(np.linspace(0, 1, n_horizon_pts))
                            
                            self._horizon_markers = ax3d.scatter(
                                p_pred[:n_horizon_pts, 0],
                                p_pred[:n_horizon_pts, 1], 
                                p_pred[:n_horizon_pts, 2],
                                c=colors,
                                s=30,
                                alpha=0.7,
                                marker='o',
                                edgecolors='k',
                                linewidths=0.5
                            )
                    except Exception:
                        try:
                            self._pred_line.set_data([], [])
                            self._pred_line.set_3d_properties([])
                        except Exception:
                            pass

                # update real point
                try:
                    self._real_point.set_data([current_position[0]], [current_position[1]])
                    self._real_point.set_3d_properties([-current_position[2]])
                except Exception:
                    pass

                # update trail
                if trail.size != 0:
                    try:
                        self._trail_line.set_data(trail[:, 0], trail[:, 1])
                        self._trail_line.set_3d_properties(trail[:, 2])
                    except Exception:
                        pass

                # dynamic goal handling
                if self.goal is not None:
                    try:
                        gx, gy, gz = float(self.goal[0]), float(self.goal[1]), -float(self.goal[2])
                        if self._goal_handle is not None:
                            try:
                                self._goal_handle.remove()
                            except Exception:
                                pass
                            self._goal_handle = None
                        if self._goal_text is not None:
                            try:
                                self._goal_text.remove()
                            except Exception:
                                pass
                            self._goal_text = None
                        self._goal_handle = ax3d.scatter([gx], [gy], [gz], marker='*', s=200, c='red', label='Goal')
                        self._goal_text = ax3d.text(gx, gy, gz, 'Goal', color='red', fontsize=9)
                    except Exception:
                        pass

                # Update velocity vector
                if current_velocity is not None:
                    try:
                        if self._velocity_vector is not None:
                            try:
                                self._velocity_vector.remove()
                            except Exception:
                                pass
                        
                        vel_scale = 2.0
                        vel_scaled = current_velocity * vel_scale
                        
                        if np.linalg.norm(vel_scaled) > 0.01:
                            self._velocity_vector = ax3d.quiver(
                                current_position[0], current_position[1], -current_position[2],
                                vel_scaled[0], vel_scaled[1], -vel_scaled[2],
                                color='blue', arrow_length_ratio=0.1, linewidth=2,
                                alpha=0.8, label='Velocity'
                            )
                    except Exception as e:
                        pass

                # Update thrust vector
                if current_thrust_vector is not None:
                    try:
                        if self._thrust_vector is not None:
                            try:
                                self._thrust_vector.remove()
                            except Exception:
                                pass
                        
                        thrust_scale = 0.1
                        thrust_scaled = current_thrust_vector * thrust_scale
                        
                        if np.linalg.norm(thrust_scaled) > 0.01:
                            self._thrust_vector = ax3d.quiver(
                                current_position[0], current_position[1], -current_position[2],
                                thrust_scaled[0], thrust_scaled[1], -thrust_scaled[2],
                                color='red', arrow_length_ratio=0.1, linewidth=2,
                                alpha=0.8, label='Thrust'
                            )
                    except Exception as e:
                        pass

                # autoscale 3D axes
                try:
                    all_pts = []
                    if trail.size != 0:
                        all_pts.append(trail)
                    if x_pred_all is not None and x_pred_all.size != 0:
                        try:
                            pred_pts = x_pred_all[:, 0:3].copy()
                            pred_pts[:, 2] = -pred_pts[:, 2]
                            all_pts.append(pred_pts)
                        except Exception:
                            pass
                    if len(all_pts) > 0:
                        big = np.vstack(all_pts)
                        pad = 0.5
                        ax3d.set_xlim(big[:, 0].min() - pad, big[:, 0].max() + pad)
                        ax3d.set_ylim(big[:, 1].min() - pad, big[:, 1].max() + pad)
                        ax3d.set_zlim(big[:, 2].min() - pad, big[:, 2].max() + pad)
                except Exception:
                    pass

                # draw and save frame
                self._draw_count += 1
                if (self._draw_count % self._draw_every) == 0:
                    try:
                        # Update legend if needed
                        handles, labels = ax3d.get_legend_handles_labels()
                        has_velocity = any('Velocity' in label for label in labels)
                        has_thrust = any('Thrust' in label for label in labels)
                        
                        if (self._velocity_vector is not None and not has_velocity) or \
                           (self._thrust_vector is not None and not has_thrust):
                            ax3d.legend(loc='upper right', fontsize='small')
                        
                        fig.canvas.draw()
                        fig.canvas.flush_events()
                        
                        # Capture and save frame incrementally
                        if self._video_writer is not None:
                            try:
                                w, h = fig.canvas.get_width_height()
                                buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
                                frame = buf.reshape(h, w, 3)
                                self._video_writer.append_data(frame)
                                self._frame_count += 1
                            except Exception as e:
                                print(f"MPCVisualizer: error saving frame {self._frame_count}: {e}")
                    except Exception:
                        pass

            # Close video writer
            if self._video_writer is not None:
                try:
                    self._video_writer.close()
                    print(f"MPCVisualizer: Video saved successfully to {os.path.abspath(self.video_filename)}")
                    print(f"MPCVisualizer: Total frames: {self._frame_count}")
                except Exception as e:
                    print(f"MPCVisualizer: error closing video writer: {e}")

            # Close figure
            try:
                plt.close(fig)
            except Exception:
                pass

        except Exception as e_main:
            print("MPCVisualizer thread error:", e_main)
            if self._video_writer is not None:
                try:
                    self._video_writer.close()
                except:
                    pass
            try:
                plt.close('all')
            except Exception:
                pass