# 011_Hip Extension

Measure and train left or right hip extension angle during walking or daily movement.

### Nodes Required: 4
- **Sensing (2)**:
  - pelvis (back center, switch pointing up)
  - thigh (left or right lateral, switch pointing up)
- **Feedback (2)**:
  - feedback_min
  - feedback_max

---

## Algorithm & Calibration

### Algorithm Information
The app uses quaternion data from IMUs to calculate 3D joint angles. The orientation is converted into XYZ Euler angles, from which the sagittal plane (hip extension) angle is extracted. This angle reflects relative rotation between the thigh and pelvis segments.

### Calibration Process
On the first iteration, the app aligns sensor orientation to body segment alignment:
- Calculates yaw offset between pelvis and thigh
- Applies this alignment to correct global measurements
- Uses predefined target orientation as a reference

---

## Description of Data in Downloaded File

### Calculated Fields
- **time (sec)**: time since trial start
- **Hip_ext (deg)**: hip flexion/extension angle
- **Feedback_min**: minimum threshold used for feedback
- **Feedback_max**: maximum threshold used for feedback

### Sensor Raw Data Fields (repeated for each sensor)
- SensorIndex
- AccelX/Y/Z (m/s²)
- GyroX/Y/Z (deg/s)
- MagX/Y/Z (μT)
- Quat1/2/3/4 (quaternion, scalar first order)
- Sampletime (timestamp)
- Package (packet number)


## Development Notes
The best place to start when developing a new app or modifying an existing one is the [SageMotion Documentation](http://docs.sagemotion.com/index.html) page.