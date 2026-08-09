bool[4] enable [0, 0, 0, 0]
# Drive refused the CiA-402 enable/disable transition: joint is not under control
bool[4] enable_fault [0, 0, 0, 0]
bool[4] encoder [0, 0, 0, 0]
bool[4] reached [0, 0, 0, 0] 
bool engaged 0
bool ready_to_engage 0
int64 locked 0
bool head_attached 0
bool trans_limit_en 0
int64 control_mode
bool procedure

float64[4] min_pos_limit [0.0, 0.0, 0.0, 0.0] 
float64[4] max_pos_limit [0.0, 0.0, 0.0, 0.0] 

float64[4] cpu_temp [0.0, 0.0, 0.0, 0.0] 
float64[4] winding_temp [0.0, 0.0, 0.0, 0.0] 
