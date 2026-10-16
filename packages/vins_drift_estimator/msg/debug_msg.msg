std_msgs/Header header

string commit_hash

time gate_pose_stamp
duration correction_duration
duration from_image_to_correction_duration

int16   correction_noise_method 
float64 correction_noise_x
float64 correction_noise_y
float64 correction_noise_z
float64 correction_noise_yaw

int16   saturation_method 
float64 saturate_value_x
float64 saturate_value_y
float64 saturate_value_z
float64 saturate_value_yaw

float64 z_x
float64 z_y
float64 z_z
float64 z_yaw

float64 innovation_x
float64 innovation_y
float64 innovation_z
float64 innovation_yaw

bool discarding_correction_due_confidence
bool discarding_correction_due_invalid_measurement
float64 confidence 
float64 confidence_threshold 

bool skipping_correction_x
bool skipping_correction_y
bool skipping_correction_z
bool skipping_correction_yaw
  
float64 mean_projection_error
float64 number_of_gates
float64 largest_gate_size

