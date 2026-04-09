import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import os

from gate_detection.models.gatenet import GateNet, IMG_SIZE, DEVICE
from gate_detection.models.quadgate import detect_corners

class GateDetectorNode(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        
        self.declare_parameter('checkpoint', '')
        self.declare_parameter('camera_topic', '/cam0/image_raw')
        self.declare_parameter('output_topic', '/gate_detection/drone_pose')
        
        checkpoint_path = self.get_parameter('checkpoint').get_parameter_value().string_value
        if not checkpoint_path:
            # Try to find it in the package share directory
            from ament_index_python.packages import get_package_share_directory
            checkpoint_path = os.path.join(
                get_package_share_directory('gate_detection'),
                'checkpoints', 'best_gatenet.pth')
        
        self.get_logger().info(f'Loading checkpoint: {checkpoint_path}')
        
        self.model = GateNet(f=4).to(DEVICE)
        checkpoint = torch.load(checkpoint_path, map_location=DEVICE, weights_only=True)
        self.model.load_state_dict(checkpoint['model'])
        self.model.eval()
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').get_parameter_value().string_value,
            self.camera_callback,
            10)
        
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('output_topic').get_parameter_value().string_value,
            10)
            
        # Default Gate Dimensions (from ArUco config)
        # Assuming gate is centered at (0,0) in its own frame
        half_w = 0.85
        h_top = 1.8
        h_bot = 0.25
        self.gate_points_3d = np.array([
            [0.0,  half_w, h_top], # outer_TL
            [0.0, -half_w, h_top], # outer_TR
            [0.0, -half_w, h_bot], # outer_BR
            [0.0,  half_w, h_bot], # outer_BL
            # Inner corners (approximate for now)
            [0.0,  half_w-0.1, h_top-0.1], 
            [0.0, -half_w+0.1, h_top-0.1],
            [0.0, -half_w+0.1, h_bot+0.1],
            [0.0,  half_w-0.1, h_bot+0.1]
        ], dtype=np.float32)

        # TODO: Load camera intrinsics from parameters or topic
        self.camera_matrix = np.array([
            [400.0, 0.0, 320.0],
            [0.0, 400.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)

    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        img_input = cv2.resize(cv_image, (IMG_SIZE, IMG_SIZE))
        img_t = torch.from_numpy(img_input).permute(2, 0, 1).float() / 255.0
        
        with torch.no_grad():
            pred = self.model(img_t.unsqueeze(0).to(DEVICE))[-1]
        
        pred_np = pred.squeeze().cpu().numpy()
        
        # Placeholder for priors (in a real scenario, these would come from EKF)
        # For now, we use dummy priors centered in the image
        priors = np.array([
            [100, 100], [284, 100], [284, 284], [100, 284],
            [120, 120], [264, 120], [264, 264], [120, 264]
        ], dtype=np.float32)
        
        refined_corners = detect_corners(pred_np, priors)
        
        if refined_corners is not None:
            # Rescale corners back to original image size
            h, w = cv_image.shape[:2]
            refined_corners[:, 0] *= (w / IMG_SIZE)
            refined_corners[:, 1] *= (h / IMG_SIZE)
            
            # Solve PnP
            success, rvec, tvec = cv2.solvePnP(
                self.gate_points_3d, 
                refined_corners, 
                self.camera_matrix, 
                self.dist_coeffs)
            
            if success:
                # Publish Pose
                # This is a WIP implementation; real coordinate transforms needed
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header = msg.header
                pose_msg.pose.pose.position.x = float(tvec[0])
                pose_msg.pose.pose.position.y = float(tvec[1])
                pose_msg.pose.pose.position.z = float(tvec[2])
                # Convert rvec to quaternion...
                
                self.publisher.publish(pose_msg)
                # self.get_logger().info('Published gate pose correction')

def main(args=None):
    rclpy.init(args=args)
    node = GateDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
