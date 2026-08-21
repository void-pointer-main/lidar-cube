# Lidar Cube
Projects readings from 12 [VL53L7CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l7cx.html) TOF sensors to 6 WS2812B LED panels of a cube.

## Projection
The projection is very simple - the 8x8 distance readings from a single lidar are directly blitted onto their surrounding regions of LED pixels.
This results in multiple distance values (specifically 2) being assigned to a LED pixel - we choose the closest one.
The distances are converted to RGB values using [hsluv-c](https://github.com/hsluv/hsluv-c).

An alternative way of projecting the distance readings to the LEDs would be to:
1. Create a point cloud out of the distance readings
2. Project the points onto the surface of the panels using linear algebra
3. Assign color to the LED pixels by sampling from the projected points (for ex. by averaging out neighbouring values, by interpolating etc.)

I chose not to go on this path, as the resolution of the lidars is already quite low, and this method would likely exacerbate this problem. 

# Other Modes
It would be too boring if we didn't add some other functionality.
Switching between modes is done by thresholding the gyroscope on a [ISM330IS](https://www.st.com/en/mems-and-sensors/ism330is.html) IMU.

## Game Of Life simulation
We can threshold the projected distance values to bring cells to life in the grid formed by the LED panels.

## Membrane simulation
By [simulating the wave equation in 2D](https://beltoforion.de/en/recreational_mathematics/2d-wave-equation.php), we can achieve an interesting look.
The membrane is actuated by incorporating the delta of the projected distance values (in effect, the velocity of the lidar distance values) into the discrete wave equation.


