#include "dac_algorithms/melee_F1.hpp"
#include "communication_protocols/joybus.hpp"
#include <math.h> 
namespace DACAlgorithms {
namespace AnalogPress {

#define coord(x) ((uint8_t)(128. + 80.*x + 0.5))
#define oppositeCoord(x) -((uint8_t)x)

struct Coords {
    uint8_t x;
    uint8_t y;
};

Coords coords(float xFloat, float yFloat) {
    Coords r;
    r.x = coord(xFloat);
    r.y = coord(yFloat);
    return r;
}

// Analog Press functions
struct int_Coords {
    int x;
    int y;
};

// button press combinations result in the cursor moving from its current position to the target
int_Coords target(bool left,bool right,bool up, bool down){
    int_Coords xy;
    if (left && up) xy = {-56,56}; 
    else if (left && down) xy = {-56,-56};
    else if (right && up) xy = {56,56};
    else if (right && down) xy = {56,-56}; 
    else if (left) xy = {-81,0}; 
    else if (right) xy = {81,0};
    else if (up) xy = {0,81};
    else if (down) xy = {0,-81}; 
    else xy ={0,0};
    return xy;
}

// The coordinate circle is divided into 9 regions. Needed for logic when rolling stick
// 4 Cardinals, 4 diagonals and the center
// Ordered starting at 1 from the East cardinal, and increasing counterclockwise. the center is -1.
uint8_t get_region(int_Coords p){
    uint8_t region;
    if (p.x >= 23){
        if (p.y >= 23) region = 2;
      else if (p.y >= -22) region = 1;
      else region = 8;
    }
    else if (p.x >= -22){
        if (p.y >= 23) region = 3;
        else if (p.y >= -22) region = -1;
        else region = 7;
    }
    else{
        if (p.y >= 23) region = 4;
        else if (p.y >= -22) region = 5;
        else region = 6;
    }
    return region;
}

// Can only send integers 0-255 as result, so need to truncate final value for report
// Save truncation and accumulate, because there are situations when dx or dy < 1
// If there was no accumulation, cursor would get stuck
int_Coords quantize(float dx,float dy,int_Coords xy,int_Coords target_point, float &dx_accum, float &dy_accum){
    int x,y;
    bool adjusted = false;
    float adjust_x, adjust_y, theta;
    int_Coords p;
    if (abs(dx)> 0 && abs(dx) < 1) dx_accum += dx;
    if (abs(dy)> 0 && abs(dy) < 1) dy_accum += dy;
    if (abs(dx_accum) > 1){
        x=xy.x+copysign(floor(abs(dx_accum)),dx_accum);
        dx_accum -= copysign(floor(abs(dx_accum)),dx_accum);
    }
    else{
        adjust_x = (abs(dx) > abs(target_point.x-xy.x)) ? target_point.x-xy.x : dx;
        x= xy.x+copysign(floor(abs(adjust_x)),adjust_x);
        if (copysign(floor(abs(adjust_x)),adjust_x) > 0) adjusted = true;
    }
    if (abs(dy_accum) > 1){
        y=xy.y+copysign(floor(abs(dy_accum)),dy_accum);
        dy_accum -= copysign(floor(abs(dy_accum)),dy_accum);
    }
    else{
        adjust_y = (abs(dy) > abs(target_point.y-xy.y)) ? target_point.y-xy.y : dy;
        y= xy.y+copysign(floor(abs(adjust_y)),adjust_y);
        if (copysign(floor(abs(adjust_y)),adjust_y) > 0) adjusted = true;
    }
    if (adjusted && (x*x+y*y)>6400){
        theta =  atan2f(y,x);
        if (abs(theta)>M_PI_4 && abs(theta)<3*M_PI_4) y -= copysign(1,y);
        else x -= copysign(1,x);
    }
    p = {x,y};    
    return p;
}

// Determine if the cursor should be moving cw or ccw when rolling
bool direction_of_change(int_Coords xy, int_Coords target_point, uint8_t region){
    bool counter_clockwise;
    if (region == 1 || region == 2 || region == 8) counter_clockwise = xy.y < target_point.y; 
    else if (region == 4 || region == 5 || region == 6) counter_clockwise = xy.y > target_point.y; 
    else if (region == 3) counter_clockwise =  xy.x > target_point.x;
    else counter_clockwise = xy.x < target_point.x;
    return counter_clockwise;
}

// Convert target coords to appropriate quadrant
int_Coords region_coords(int_Coords xy, uint8_t region){
    int_Coords p;
    switch (region){
        case 2:
            p = {xy.x,xy.y};
            break;
        case 4:
            p = {-1*xy.x,xy.y};
            break;
        case 6: 
            p = {-1*xy.x,-1*xy.y};
            break;
        case 8:
            p = {xy.x,-1*xy.y};
            break;
        default:
            p = {xy.x,xy.y};
    }
    return p;
}

// Find central angle between two points on circle
float angle_to_target(int_Coords xy,int_Coords target_point){
    float theta;
    if ((xy.x == 0 && target_point.x == 0) || (xy.y==0 && target_point.y==0)) theta = 0;
    else theta = acosf((12800 - ((xy.x-target_point.x)*(xy.x-target_point.x)+(xy.y-target_point.y)*(xy.y-target_point.y)))/12800.0);
  return theta;
}

// Given an angle travelled, determine coordinates of new point
int_Coords roll_to_new_point(int_Coords xy, float theta,float current_theta,float &dx_accum, float &dy_accum){
    int_Coords p;
    float dx = 80.0*(cosf(theta) - cosf(current_theta));
    float dy = 80*(sinf(theta) - sinf(current_theta));
    int_Coords target_p = {xy.x+int(dx),xy.y+int(dy)};
    p = quantize(dx,dy,xy,target_p,dx_accum,dy_accum);
    return p;
}
// Analog Press declarations
uint32_t t = time_us_32();
uint32_t dt;
uint8_t VEL_FAST = 50;
uint8_t VEL_SLOW = 5;
uint8_t VEL_RETURN = 25;
uint8_t VEL_ROLL = 10;
uint8_t v;
float d;
float dx;
float dy;
float dx_accum=0;
float dy_accum=0;
float theta;
float new_theta;
float d_theta;
float current_theta;
int_Coords xy = {0,0};
int_Coords target_point;
int_Coords new_point;
uint8_t current_region;
uint8_t target_region;
bool no_direction;
bool roll_stick;
bool adjacent_region;
bool counter_clockwise;
bool reset_hold =true;
// 2 IP declarations
bool left_wasPressed = false;
bool right_wasPressed = false;
bool up_wasPressed = false;
bool down_wasPressed = false;

bool left_outlawUntilRelease = false;
bool right_outlawUntilRelease = false;
bool up_outlawUntilRelease = false;
bool down_outlawUntilRelease = false;

GCReport getGCReport(GpioToButtonSets::F1::ButtonSet buttonSet) {
    
    GpioToButtonSets::F1::ButtonSet bs = buttonSet; // Alterable copy

    GCReport gcReport = defaultGcReport;

    /* 2IP No reactivation */
    if (left_wasPressed && bs.left && bs.right && !right_wasPressed) left_outlawUntilRelease=true;
    if (right_wasPressed && bs.left && bs.right && !left_wasPressed) right_outlawUntilRelease=true;
    if (up_wasPressed && bs.up && bs.down && !down_wasPressed) up_outlawUntilRelease=true;
    if (down_wasPressed && bs.up && bs.down && !up_wasPressed) down_outlawUntilRelease=true;

    if (!bs.left) left_outlawUntilRelease=false;
    if (!bs.right) right_outlawUntilRelease=false;
    if (!bs.up) up_outlawUntilRelease=false;
    if (!bs.down) down_outlawUntilRelease=false;

    left_wasPressed = bs.left;
    right_wasPressed = bs.right;
    up_wasPressed = bs.up;
    down_wasPressed = bs.down;

    if (left_outlawUntilRelease) bs.left=false;
    if (right_outlawUntilRelease) bs.right=false;
    if (up_outlawUntilRelease) bs.up=false;
    if (down_outlawUntilRelease) bs.down=false;
    
    /* Stick */

    bool readUp = bs.up;
    //Start Analog Press
    dt = time_us_32() - t;
    target_point = target(bs.left,bs.right,bs.up,bs.down);
    no_direction = !bs.left && !bs.right && !bs.up && !bs.down;
    current_region = get_region(xy);
    target_region = get_region(target_point);
    adjacent_region = abs(target_region-current_region) <=1 ||  target_region + current_region == 9;
    roll_stick = (xy.x*xy.x+xy.y*xy.y)>=5625 && adjacent_region;
    if (no_direction) v=VEL_RETURN;
    else if (bs.my) v=VEL_SLOW;
    else if (roll_stick) v=VEL_ROLL;
    else v=VEL_FAST;
    d = v*dt/10000.0;
    if (!bs.ms) reset_hold=true;
    // Holding
    if (bs.ms){
        // Return to origin if all buttons released. Stays fix until hold is then released
        if(no_direction || !reset_hold){
            target_point = {0,0};
            // 10000 because velocity is units/10ms and t is in us
            // used units/10ms because I wanted to avoid v being decimals 
            // eg. V_SlOW is 5, which represents 0.5 units/ms. Didn't want to store as 0.5
            d = VEL_RETURN*dt/10000.0;
            theta = atan2f(target_point.y-xy.y,target_point.x-xy.x);
            dx = (target_point.x == xy.x) ? 0 : d*cosf(theta);
            dy = d*sinf(theta);
            new_point = quantize(dx,dy,xy,target_point,dx_accum,dy_accum);
            xy.x = new_point.x;
            xy.y = new_point.y;
            reset_hold= false;
        }
    }
    // Rollling
    else if (roll_stick){
        theta = d/(80.0);
        counter_clockwise = direction_of_change(xy,target_point,current_region);
        current_theta = atan2f(xy.y,xy.x);
        new_theta = (counter_clockwise) ? current_theta + theta : current_theta - theta;
        if (new_theta > M_PI) new_theta -= 2*M_PI;
        else if (new_theta < -1*M_PI) new_theta += 2*M_PI;
        // Notches 
        if (bs.mx && current_region%2==0 && target_region%2==1){
            // Vertical
            if (((current_region == 2 || current_region == 4) && target_region == 3) || ((current_region == 6 || current_region == 8) && target_region == 7)) target_point = region_coords({31,73},current_region);
            // Horizontal
            else target_point = region_coords({73,31},current_region);
            d_theta = angle_to_target(xy,target_point);
            if (theta >= abs(d_theta) || xy.x == target_point.x || xy.y == target_point.y){
                xy.x = target_point.x;
                xy.y = target_point.y;
            }
            else xy = roll_to_new_point(xy,new_theta,current_theta ,dx_accum,dy_accum);
        }
        // Shield Drop
        else if ((bs.l|| bs.r || bs.ls) && target_region%2==0 &&xy.y>target_point.y){
            target_point = region_coords({56,55},target_region);
            d_theta = angle_to_target(xy,target_point);
            if (theta >= abs(d_theta) || xy.x == target_point.x || xy.y == target_point.y){
                xy.x = target_point.x;
                xy.y = target_point.y;
            }
            else xy = roll_to_new_point(xy,new_theta,current_theta ,dx_accum,dy_accum);
        }
        // Roll to default coordinate
        else{
            d_theta = angle_to_target(xy,target_point);
            if (theta >= abs(d_theta) || xy.x == target_point.x || xy.y == target_point.y){
                xy.x = target_point.x;
                xy.y = target_point.y;
            }
            else xy = roll_to_new_point(xy,new_theta,current_theta ,dx_accum,dy_accum);
        }
    }
    // Default
    // Move from current position to target in a straight line
    else{
        theta = atan2f(target_point.y-xy.y,target_point.x-xy.x);
        dx = (target_point.x == xy.x) ? 0 : d*cosf(theta);
        dy = d*sinf(theta);
        new_point = quantize(dx,dy,xy,target_point,dx_accum,dy_accum);
        if ((new_point.x*new_point.x)+(new_point.y*new_point.y)>6724){
            xy.x = target_point.x;
            xy.y = target_point.y;
        }
        else{
            xy.x = new_point.x;
            xy.y = new_point.y;
        }
    }
    gcReport.xStick = uint8_t(xy.x+128);
    gcReport.yStick = uint8_t(xy.y+128);
    //End Analog Press

    /* C-Stick */
    
    bool cVertical = bs.cUp != bs.cDown;
    bool cHorizontal = bs.cLeft != bs.cRight;

    Coords cxy;

    if (bs.mx && bs.my) cxy = coords(0.0, 0.0);
    else if (cVertical && cHorizontal) cxy = coords(0.525, 0.85);
    else if (cHorizontal) cxy = bs.mx ? coords(0.8375, readUp ? 0.3125 : -0.3125) : coords(1.0, 0.0);
    else if (cVertical) cxy = coords(0.0, 1.0);
    else cxy = coords(0.0, 0.0);

    if (cHorizontal && bs.cLeft) cxy.x = oppositeCoord(cxy.x);
    if (cVertical && bs.cDown) cxy.y = oppositeCoord(cxy.y);

    gcReport.cxStick = cxy.x;
    gcReport.cyStick = cxy.y;

    /* Dpad */
    if (bs.mx && bs.my) {
        gcReport.dDown = bs.cDown;
        gcReport.dLeft = bs.cLeft;
        gcReport.dUp = bs.cUp;
        gcReport.dRight = bs.cRight;
    }

    /* Triggers */
    gcReport.analogL = bs.l ? 140 : bs.ls ? 49 : 0;
    gcReport.analogR = bs.r ? 140 : 0;

    /* Buttons */
    gcReport.a = bs.a;
    gcReport.b = bs.b;
    gcReport.x = bs.x;
    gcReport.y = bs.y;
    gcReport.z = bs.z;
    gcReport.l = bs.l;
    gcReport.r = bs.r;
    gcReport.start = bs.start;
    //Timing for Analog press
    t = time_us_32();
    return gcReport;
}

}
}