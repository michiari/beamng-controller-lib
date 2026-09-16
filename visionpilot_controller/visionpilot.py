from pathlib import Path
import time
import yaml
import cv2
import numpy as np

from config.config import BASE_DIR
from utils.pid_controller import PIDController
from src.perception.lane_detection.main import process_frame_cv as cv_lane_process
from beamngpy.sensors import Camera

from beamng_controllers.controller_wrapper import ControllerWrapper

DEFAULT_BEAMNG_STEPS_PER_SECOND = 60
DEFAULT_CV_HINT_NUM_LANES = 3

class VisionPilotController(ControllerWrapper):
    """A controller that uses a VisionPilot model to make driving decisions."""

    def __init__(
            self,
            beamng_client,
            vehicle,
            beamng_steps_per_second=DEFAULT_BEAMNG_STEPS_PER_SECOND,
            cv_hint_num_lanes=DEFAULT_CV_HINT_NUM_LANES,
            debug_show_cv_lane_detection_output=False,
            debug_cv_lane_detection=False,
            debug_perspective=False
        ):
        self.vehicle = vehicle
        self.beamng_steps_per_second = beamng_steps_per_second
        self.cv_hint_num_lanes = cv_hint_num_lanes
        self.debug_show_cv_lane_detection_output = debug_show_cv_lane_detection_output
        self.debug_cv_lane_detection = debug_cv_lane_detection
        self.debug_perspective = debug_perspective
        # Load control parameters from config
        self.sensors_config, control, perception_config = self._load_visionpilot_config()
        self.control_cfg = control['control']
        self.perception_cfg = perception_config['perception']

        # Enable debug for steering PID to see terms
        steering_pid_config = self.control_cfg['steering_pid'].copy()
        steering_pid_config['debug'] = True
        steering_pid = PIDController(**steering_pid_config)
        self.steering_pid = steering_pid
        self.max_steering_change = self.control_cfg['max_steering_change']
        self.previous_steering = 0.0
        self.target_speed_kph = self.control_cfg['target_speed_kph']
        self.speed_pid = PIDController(
            Kp=self.control_cfg['speed_pid']['Kp'],
            Ki=self.control_cfg['speed_pid']['Ki'],
            Kd=self.control_cfg['speed_pid']['Kd']
        )
        self.speed_control_mode = self.control_cfg['speed_control_mode']
        print(f"[Main] Speed control mode: {self.speed_control_mode}")

        self._setup_sensors(beamng_client)

        self.previous_steering = 0.0


    def _setup_sensors(self, beamng_client):
        # Setup sensors - select config based on vehicle model
        vehicle_model = self.vehicle.model
        if vehicle_model not in self.sensors_config:
            raise ValueError(f"Sensor configuration for vehicle model '{vehicle_model}' not found in config")
        
        sensors = self.sensors_config[vehicle_model]
        sensor_key = 'camera_front'
        if sensor_key not in sensors:
            raise ValueError(f"Front camera configuration for vehicle model '{vehicle_model}' not found in config")
        sensor_cfg = sensors[sensor_key]
        self.camera_front = Camera(
            sensor_cfg['name'],
            beamng_client,
            self.vehicle,
            requested_update_time=sensor_cfg['requested_update_time'],
            is_using_shared_memory=sensor_cfg.get('is_using_shared_memory', False),
            pos=tuple(sensor_cfg['pos']),
            dir=tuple(sensor_cfg['dir']),
            field_of_view_y=sensor_cfg['field_of_view_y'],
            near_far_planes=tuple(sensor_cfg['near_far_planes']),
            resolution=tuple(sensor_cfg['resolution']),
            is_streaming=sensor_cfg.get('is_streaming', False),
            is_render_colours=sensor_cfg.get('is_render_colours', True),
            is_render_depth=sensor_cfg.get('is_render_depth', False),
            is_visualised=sensor_cfg.get('is_visualised', False),
        )
        print(f"Camera '{sensor_key}' initialized")

    def next_control(self, step_size=10):
        speed_mps, speed_kph, car_pos, direction = self._get_vehicle_state()

        images = self.camera_front.poll()
        img = np.array(images['colour'], dtype=np.uint8)

        # Lane Detection
        try:
            start_proc = time.time()
            
            # # conditional execution based on perception flags
            # if enable_obj_det:
            #     object_detections, _ = object_process(img_bgr, confidence_threshold=0.4, draw_detections=False, model=local_models.get('vehicle'))
            # else:
            #     object_detections = []

            cv_result_image, lane_metrics, cv_confidence = cv_lane_process(
                img,
                speed=speed_kph,
                previous_steering=self.previous_steering,
                debug_display=self.debug_cv_lane_detection,
                perspective_debug_display=self.debug_perspective,
                vehicle_model=self.vehicle.model,
                num_lanes=self.cv_hint_num_lanes
            )

            print(f"Local processing latency: {(time.time()-start_proc)*1000:.1f}ms")
            
            deviation = lane_metrics.get('deviation', 0.0)
            # ensure deviation is not None
            if deviation is None:
                deviation = 0.0
            smoothed_deviation = lane_metrics.get('smoothed_deviation', deviation)
            if smoothed_deviation is None:
                smoothed_deviation = deviation
            effective_deviation = lane_metrics.get('effective_deviation', deviation)
            if effective_deviation is None:
                effective_deviation = deviation
            lane_center = lane_metrics.get('lane_center', 0.0)
            if lane_center is None:
                lane_center = 0.0
            vehicle_center = lane_metrics.get('vehicle_center', 0.0)
            if vehicle_center is None:
                vehicle_center = 0.0
            fused_confidence = lane_metrics.get('confidence', 0.0)
            if fused_confidence is None:
                fused_confidence = 0.0
            
            # Log lane tracking info
            current_lane = lane_metrics.get('current_lane')
            if current_lane:
                print(f"[MAIN] Tracking: lane={current_lane.get('lane_class', '?')} (ID:{current_lane.get('lane_id', '?')}), pos_in_lane={current_lane.get('position_in_lane', 0):.2f}, deviation={deviation:.3f}m, eff_dev={effective_deviation:.3f}m")
            else:
                print(f"[MAIN] No lane tracked, deviation={deviation:.3f}m, eff_dev={effective_deviation:.3f}m")
        
        except Exception as agg_e:
            print(f"[Main] Local perception error: {agg_e}")
            return

        SIM_DT = step_size / float(self.beamng_steps_per_second)
        steering = self.steering_pid.update(-effective_deviation, SIM_DT)
        steering = np.clip(steering, -1.0, 1.0)
        steering_change = steering - self.previous_steering
        if abs(steering_change) > self.max_steering_change:
            steering = self.previous_steering + np.sign(steering_change) * self.max_steering_change

        throttle = 0.0
        brake = 0.0

        # Normal cruise control (no adaptive features)
        throttle = self._cruise_control(self.target_speed_kph, speed_kph, self.speed_pid, SIM_DT)
        brake = 0.0

        # Limit throttle based on steering angle to prevent spinning out
        # if speed_control_mode == 'none':
        #     throttle = 0.0
        # else:
        throttle *= (1.0 - 0.3 * abs(steering))
        throttle = float(np.clip(throttle, 0.0, 0.3))
        
        #Application of the vehicle controls to BeamNG

        try:
            self.vehicle.control(throttle=float(throttle), brake=float(brake), steering=float(steering))
        except Exception as e:
            print(f"[Main] Error sending control to vehicle: {e}")
            
        self.previous_steering = steering

        # Display CV lane detection window
        if self.debug_show_cv_lane_detection_output and cv_result_image is not None:
            cv_disp = cv2.cvtColor(cv_result_image, cv2.COLOR_RGB2BGR) if len(cv_result_image.shape) == 3 else cv_result_image
            cv_disp = cv2.resize(cv_disp, (0, 0), fx=0.25, fy=0.25)
            cv2.imshow('CV Lane Detection', cv_disp)
            cv2.waitKey(1)


    def _get_vehicle_state(self):
        """
        Get the vehicle position and speed in m/s and kph.
        Returns:
            tuple: (speed_mps, speed_kph, position)
        """

        self.vehicle.poll_sensors()
        if 'vel' in self.vehicle.state:
            speed_mps = abs(self.vehicle.state['vel'][0])
            speed_kph = abs(speed_mps * 3.6)
        else:
            speed_mps = 0.0
            speed_kph = 0.0

        position = self.vehicle.state.get('pos', None)
        direction = self.vehicle.state.get('dir', None)

        return speed_mps, speed_kph, position, direction

    def _cruise_control(self,target_speed_kph, current_speed_kph, speed_pid, SIM_DT):
        """
        Simple cruise control to maintain target speed using PID controller.
        Args:
            target_speed_kph (float): Desired speed in kph
            current_speed_kph (float): Current speed in kph
            speed_pid (PIDController): PID controller instance for speed
            SIM_DT (float): Simulation time delta in seconds
        Returns:
            float: Throttle value between 0.0 and 1.0
        """
        speed_error = target_speed_kph - current_speed_kph
        throttle = speed_pid.update(speed_error, SIM_DT)
        throttle = np.clip(throttle, 0.0, 1.0)
        return throttle

    def _load_visionpilot_config(self):
        """Load VisionPilot configuration files."""
        config_path = BASE_DIR / "config"

        with open(config_path / 'sensors.yaml', 'r') as f:
            sensors_config = yaml.safe_load(f)
        with open(config_path / 'control.yaml', 'r') as f:
            control = yaml.safe_load(f)
        with open(config_path / 'perception.yaml', 'r') as f:
            perception_config = yaml.safe_load(f)

        return sensors_config, control, perception_config
