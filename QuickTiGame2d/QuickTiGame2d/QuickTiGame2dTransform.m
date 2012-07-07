// Copyright (c) 2012 quicktigame2d project
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the project nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
#import "QuickTiGame2dTransform.h"
#import "QuickTiGame2dConstant.h"
#import "QuickTiGame2dEngine.h"


@implementation QuickTiGame2dTransform
@synthesize x, y, z, width, height, delay, duration, repeat, easing, frameIndex, angle, startTime;
@synthesize rotate_axis, rotate_centerX, rotate_centerY, scaleX, scaleY, red, green, blue, alpha, repeatCount;
@synthesize scale_centerX, scale_centerY;

@synthesize start_x, start_y, start_z, start_width, start_height, start_frameIndex, start_angle;
@synthesize start_rotate_axis, start_rotate_centerX, start_rotate_centerY, start_scaleX;
@synthesize start_scaleY, start_red, start_green, start_blue, start_alpha;

@synthesize current_x, current_y, current_z, current_width, current_height, current_frameIndex, current_angle;
@synthesize current_rotate_axis, current_rotate_centerX, current_rotate_centerY, current_scaleX;
@synthesize current_scaleY, current_red, current_green, current_blue, current_alpha;

@synthesize autoreverse, reversing, completed;

@synthesize bezier, bc1x, bc1y, bc2x, bc2y;

- (id)init {
    self = [super init];
    if (self != nil) {
        easing = ANIMATION_EASING_LINEAR;
        repeatCount = 0;
        
        autoreverse = FALSE;
        reversing   = FALSE;
        
        completed   = FALSE;
    }
    return self;
}

- (void)dealloc {
    [x release];
    [y release];
    [z release];
    [width release];
    [height release];
    
    [frameIndex release];
    
    [angle release];
    [rotate_axis release];
    [rotate_centerX release];
    [rotate_centerY release];
    
    [scaleX release];
    [scaleY release];
    
    [red release];
    [green release];
    [blue release];
    [alpha release];
    
    [super dealloc];
}

-(NSInteger)elapsedFromStart {
    return ([QuickTiGame2dEngine uptime] - startTime) * 1000;
}

-(NSInteger)elapsed {
    return [self elapsedFromStart] - delay;
}

-(BOOL)hasExpired {
    return [self elapsed] >= duration;
}

-(BOOL)hasStarted {
    return [self elapsed] > 0;
}

-(void)start {
    startTime   = [QuickTiGame2dEngine uptime];
    reversing   = FALSE;
    repeatCount = 0;
    completed   = FALSE;
}

-(void)restart {
    repeatCount++;
    startTime = [QuickTiGame2dEngine uptime];
}

-(void)reverse {
    if (reversing) {
        repeatCount++;
    }
    startTime = [QuickTiGame2dEngine uptime];
    reversing = !reversing;
}

-(void)apply {
    if (![self hasStarted]) return;
    
    if (bezier) {
        current_x = [self current_bezier_x:start_x to:[x floatValue]];
        current_y = [self current_bezier_y:start_y to:[y floatValue]];
    } else {
        current_x = [self current:start_x to:[x floatValue]];
        current_y = [self current:start_y to:[y floatValue]];
    }
    
    
    
    current_z = [self current:start_z to:[z floatValue]];
    current_width  = [self current:start_width to:[width intValue]];
    current_height = [self current:start_height to:[height intValue]];
    current_frameIndex = [self current:start_frameIndex to:[frameIndex intValue]];
    
    current_angle = [self current:start_angle to:[angle floatValue]];
    current_rotate_axis = [self current:start_rotate_axis to:[rotate_axis floatValue]];
    current_rotate_centerX = [self current:start_rotate_centerX to:[rotate_centerX floatValue]];
    current_rotate_centerY = [self current:start_rotate_centerY to:[rotate_centerY floatValue]];
    
    current_scaleX = [self current:start_scaleX to:[scaleX floatValue]];
    current_scaleY = [self current:start_scaleY to:[scaleY floatValue]];

    current_red   = [self current:start_red to:[red floatValue]];
    current_green = [self current:start_green to:[green floatValue]];
    current_blue  = [self current:start_blue to:[blue floatValue]];
    current_alpha = [self current:start_alpha to:[alpha floatValue]];

}

-(void)color:(float)_red green:(float)_green blue:(float)_blue {
    self.red   = [NSNumber numberWithFloat:_red];
    self.green = [NSNumber numberWithFloat:_green];
    self.blue  = [NSNumber numberWithFloat:_blue];
}

-(void)color:(float)_red green:(float)_green blue:(float)_blue alpha:(float)_alpha {
    self.red   = [NSNumber numberWithFloat:_red];
    self.green = [NSNumber numberWithFloat:_green];
    self.blue  = [NSNumber numberWithFloat:_blue];
    self.alpha = [NSNumber numberWithFloat:_alpha];
}

-(void)rotate:(float)_angle {
    self.angle = [NSNumber numberWithFloat:_angle];
}

-(void)rotateZ:(float)_angle {
    self.angle = [NSNumber numberWithFloat:_angle];
    self.rotate_axis = [NSNumber numberWithFloat:AXIS_Z];
}

-(void)rotateY:(float)_angle {
    self.angle = [NSNumber numberWithFloat:_angle];
    self.rotate_axis = [NSNumber numberWithFloat:AXIS_Y];
}

-(void)rotateX:(float)_angle {
    self.angle = [NSNumber numberWithFloat:_angle];
    self.rotate_axis = [NSNumber numberWithFloat:AXIS_X];
}

-(void)rotate:(float)_angle centerX:(float)_centerX centerY:(float)_centerY {
    self.angle = [NSNumber numberWithFloat:_angle];
    self.rotate_centerX = [NSNumber numberWithFloat:_centerX];
    self.rotate_centerY = [NSNumber numberWithFloat:_centerY];
}

-(void)scale:(float)scale {
    self.scaleX = [NSNumber numberWithFloat:scale];
    self.scaleY = [NSNumber numberWithFloat:scale];
}

-(void)scale:(float)_scaleX scaleY:(float)_scaleY {
    self.scaleX = [NSNumber numberWithFloat:_scaleX];
    self.scaleY = [NSNumber numberWithFloat:_scaleY];
}

-(void)move:(float)_x y:(float)_y {
    self.x = [NSNumber numberWithFloat:_x];
    self.y = [NSNumber numberWithFloat:_y];
}

-(float)current:(float)_from to:(float)_to {
    float percent = [self ease:[self elapsed] duration:self.duration];
    if ([self hasExpired]) {
        percent = reversing ? 0 : 1;
    }
    return _from + (percent * (_to - _from));
}

-(float)ease:(float)_elapsed duration:(float)_duration {
    if (reversing) {
        _elapsed = _duration - _elapsed;
    }
    if (self.easing == ANIMATION_EASING_LINEAR) {
        return [self easingLinear:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_CUBIC_IN) {
        return [self easingCubicIn:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_CUBIC_OUT) {
        return [self easingCubicOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_CUBIC_INOUT) {
        return [self easingCubicInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_BACK_IN) {
        return [self easingBackIn: _elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_BACK_OUT) {
        return [self easingBackOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_BACK_INOUT) {
        return [self easingBackInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_ELASTIC_INOUT) {
        return [self easingElasticInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_ELASTIC_IN) {
        return [self easingElasticIn: _elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_ELASTIC_OUT) {
        return [self easingElasticOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_BOUNCE_INOUT) {
        return [self easingBounceInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_BOUNCE_IN) {
        return [self easingBounceIn:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_BOUNCE_OUT) {
        return [self easingBounceOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_EXPO_INOUT) {
        return [self easingExpoInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_EXPO_IN) {
        return [self easingExpoIn:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_EXPO_OUT) {
        return [self easingExpoOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_QUAD_INOUT) {
        return [self easingQuadInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_QUAD_IN) {
        return [self easingQuadIn:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_QUAD_OUT) {
        return [self easingQuadOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_SINE_INOUT) {
        return [self easingSineInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_SINE_IN) {
        return [self easingSineIn:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_SINE_OUT) {
        return [self easingSineOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_CIRC_INOUT) {
        return [self easingCircInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_CIRC_IN) {
        return [self easingCircIn:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_CIRC_OUT) {
        return [self easingCircOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_QUINT_INOUT) {
        return [self easingQuintInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_QUINT_IN) {
        return [self easingQuintIn:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_QUINT_OUT) {
        return [self easingQuintOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_QUART_INOUT) {
        return [self easingQuartInOut:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_QUART_IN) {
        return [self easingQuartIn:_elapsed duration:_duration];
    } else if (self.easing == ANIMATION_EASING_QUART_OUT) {
        return [self easingQuartOut:_elapsed duration:_duration];
    } else {
        return [self easingLinear:_elapsed duration:_duration];
    }
}

-(float)easingLinear:(float)_elapsed duration:(float)_duration {
    return _elapsed / _duration;
}

-(float)easingCubicIn:(float)_elapsed duration:(float)_duration {
    return (_elapsed = _elapsed / _duration) * pow(_elapsed, 2);
}

-(float)easingCubicOut:(float)_elapsed duration:(float)_duration {
    return (_elapsed = _elapsed / _duration - 1) * pow(_elapsed, 2) + 1;
}

-(float)easingCubicInOut:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration / 2.0) < 1) return 1.0 / 2.0 * pow(_elapsed, 3);
    return 1.0 / 2.0 * ((_elapsed -= 2) * pow(_elapsed, 2) + 2);
}

-(float)easingBackIn:(float)_elapsed duration:(float)_duration {
    return (_elapsed /= _duration) * _elapsed * ((1.70158 + 1) * _elapsed - 1.70158);
}

-(float)easingBackOut:(float)_elapsed duration:(float)_duration {
    return ((_elapsed = _elapsed / _duration - 1) * _elapsed * ((1.70158 + 1) * _elapsed + 1.70158) + 1);
}

-(float)easingBackInOut:(float)_elapsed duration:(float)_duration {
    float s = 1.70158;
    if ((_elapsed /= _duration / 2.0) < 1) return 1.0 / 2.0 * (_elapsed * _elapsed * (((s *= (1.525)) + 1) * _elapsed - s));
    return 1.0 / 2.0 * ((_elapsed -= 2) * _elapsed * (((s *= (1.525)) + 1) * _elapsed + s) + 2);
}

-(float)easingElasticIn:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration) == 1) return 1;
    float p=  _duration * 0.3;
    float s = p / 4.0;
    return -(pow(2, 10 * (_elapsed -= 1)) * sin((_elapsed * _duration - s) * (2.0 * M_PI) / p));
}

-(float)easingElasticOut:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration) == 1) return 1;
    float p=  _duration * 0.3;
    float s = p / 4.0;
    return (pow(2, -10 * _elapsed) * sin((_elapsed * _duration - s) * (2.0 * M_PI) / p) + 1);
}

-(float)easingElasticInOut:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration / 2.0) == 2) return 1;
    float p=  _duration * (0.3 * 1.5);
    float s = p / 4.0;
    if (_elapsed < 1) return -0.5 * (pow(2, 10 * (_elapsed -= 1)) * sin((_elapsed * _duration - s) * (2.0 * M_PI) / p));
    return pow(2, -10 * (_elapsed -= 1)) * sin((_elapsed * _duration - s) * (2.0 * M_PI) / p) * 0.5 + 1;
}

-(float)easingBounceOut:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration) < (1.0 / 2.75)) return (7.5625 * _elapsed * _elapsed);
    else if (_elapsed < (2.0 / 2.75)) return (7.5625 * (_elapsed -= (1.5 / 2.75)) * _elapsed + 0.75);
    else if (_elapsed < (2.5 / 2.75)) return (7.5625 * (_elapsed -= (2.25 / 2.75)) * _elapsed + 0.9375);
    else return (7.5625 * (_elapsed -= (2.625 / 2.75)) * _elapsed + 0.984375);
}

-(float)easingBounceIn:(float)_elapsed duration:(float)_duration {
    return 1 - [self easingBounceOut:_duration - _elapsed duration:_duration];
}

-(float)easingBounceInOut:(float)_elapsed duration:(float)_duration {
    if (_elapsed < _duration / 2.0) return [self easingBounceIn:_elapsed * 2.0 duration:_duration] * 0.5;
    else return [self easingBounceOut:_elapsed * 2.0 - _duration duration:_duration] * 0.5 + 0.5;
}

-(float)easingExpoIn:(float)_elapsed duration:(float)_duration {
    return (_elapsed == 0) ? 0 : pow(2, 10 * (_elapsed / _duration - 1));
}

-(float)easingExpoOut:(float)_elapsed duration:(float)_duration {
    return (_elapsed == _duration) ? 1 : (-pow(2, -10 * _elapsed / _duration) + 1);
}

-(float)easingExpoInOut:(float)_elapsed duration:(float)_duration {
    if (_elapsed == 0) return 0;
    if (_elapsed == _duration) return 1;
    if ((_elapsed /= _duration / 2.0) < 1) return 1.0 / 2.0 * pow(2, 10 * (_elapsed - 1));
    return 1.0 / 2.0 * (-pow(2, -10 * --_elapsed) + 2);
}

-(float)easingQuadIn:(float)_elapsed duration:(float)_duration {
    return (_elapsed /= _duration) * _elapsed;
}

-(float)easingQuadOut:(float)_elapsed duration:(float)_duration {
    return -1 * (_elapsed /= _duration) * (_elapsed - 2);
}

-(float)easingQuadInOut:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration / 2.0) < 1) return 1.0 / 2.0 * _elapsed * _elapsed;
    return -1.0 / 2.0 * ((--_elapsed) * (_elapsed - 2) - 1);
}

-(float)easingSineIn:(float)_elapsed duration:(float)_duration {
    return -1 * cos(_elapsed / _duration * (M_PI / 2.0)) + 1;
}

-(float)easingSineOut:(float)_elapsed duration:(float)_duration {
    return 1 * sin(_elapsed / _duration * (M_PI / 2.0));
}

-(float)easingSineInOut:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration / 2.0) < 1) return 1.0 / 2.0 * (sin(M_PI * _elapsed / 2.0));
    return -1.0 / 2.0 * (cos(M_PI * --_elapsed / 2.0) - 2);
}

-(float)easingCircIn:(float)_elapsed duration:(float)_duration {
    return -1 * (sqrt(1 - (_elapsed /= _duration) * _elapsed) - 1);
}

-(float)easingCircOut:(float)_elapsed duration:(float)_duration {
    return sqrt(1 - (_elapsed = _elapsed / _duration - 1) * _elapsed);
}

-(float)easingCircInOut:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration / 2.0) < 1) return -1.0 / 2.0 * (sqrt(1 - _elapsed * _elapsed) - 1);
    return 1.0 / 2.0 * (sqrt(1 - (_elapsed -= 2) * _elapsed) + 1);
    
}

-(float)easingQuintIn:(float)_elapsed duration:(float)_duration {
    return (_elapsed /= _duration) * _elapsed * _elapsed * _elapsed * _elapsed;
}

-(float)easingQuintOut:(float)_elapsed duration:(float)_duration {
    return ((_elapsed = _elapsed / _duration - 1) * _elapsed * _elapsed * _elapsed * _elapsed + 1);
}

-(float)easingQuintInOut:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration / 2.0) < 1) return 1.0 / 2.0 * _elapsed * _elapsed * _elapsed * _elapsed * _elapsed;
    return 1.0 / 2.0 * ((_elapsed -= 2) * _elapsed * _elapsed * _elapsed * _elapsed + 2);
}

-(float)easingQuartIn:(float)_elapsed duration:(float)_duration {
    return (_elapsed /= _duration) * _elapsed * _elapsed * _elapsed;
}

-(float)easingQuartOut:(float)_elapsed duration:(float)_duration {
    return -1 * ((_elapsed = _elapsed / _duration - 1) * _elapsed * _elapsed * _elapsed - 1);
}

-(float)easingQuartInOut:(float)_elapsed duration:(float)_duration {
    if ((_elapsed /= _duration / 2.0) < 1) return 1.0 / 2.0 * _elapsed * _elapsed * _elapsed * _elapsed;
    return -1.0 / 2.0 * ((_elapsed -= 2) * _elapsed * _elapsed * _elapsed - 2);
}


-(void)addBc1:(float)_x y:(float)_y {
    self.bc1x = [NSNumber numberWithFloat:_x];
    self.bc1y = [NSNumber numberWithFloat:_y];
}
-(void)addBc2:(float)_x y:(float)_y {
    self.bc2x = [NSNumber numberWithFloat:_x];
    self.bc2y = [NSNumber numberWithFloat:_y];
}

        
-(float)current_bezier_x:(float)_from to:(float)_to;
{
    float t = [self ease:[self elapsed] duration:self.duration];
    if ([self hasExpired]) {
        t = reversing ? 0 : 1;
    }
    
    float qx, qy;
    float q1, q2, q3, q4;
    int plotx, ploty;
    
    q1 = t*t*t*-1 + t*t*3 + t*-3 + 1;
    q2 = t*t*t*3 + t*t*-6 + t*3;
    q3 = t*t*t*-3 + t*t*3;
    q4 = t*t*t;
    
    float resX = q1*_from + q2*[bc1x floatValue] + q3*[bc2x floatValue] + q4*_to;
    //NSLog(@"t: %f, x:%f bc1x:%f, bc2x:%f, start:%f, end:%f", t, resX, [bc1x floatValue], [bc2x floatValue], _from, _to);
    return resX;
}

-(float)current_bezier_y:(float)_from to:(float)_to;
{
    float t = [self ease:[self elapsed] duration:self.duration];
    if ([self hasExpired]) {
        t = reversing ? 0 : 1;
    }

    float qx, qy;
    float q1, q2, q3, q4;
    int plotx, ploty;
    
    q1 = t*t*t*-1 + t*t*3 + t*-3 + 1;
    q2 = t*t*t*3 + t*t*-6 + t*3;
    q3 = t*t*t*-3 + t*t*3;
    q4 = t*t*t;
    
    float resY = q1*_from + q2*[bc1y floatValue] + q3*[bc2y floatValue] + q4*_to;
    //NSLog(@"t: %f, y:%f bc1y:%f, bc2y:%f, start:%f, end:%f", t, resY, [bc1y floatValue], [bc2y floatValue], _from, _to);
    return resY;
}
@end
