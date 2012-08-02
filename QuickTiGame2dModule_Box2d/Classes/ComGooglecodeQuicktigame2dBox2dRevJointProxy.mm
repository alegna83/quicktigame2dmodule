/**
 *  Copyright 2012 QuickTiGame2d project
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
#import "ComGooglecodeQuicktigame2dBox2dRevJointProxy.hh"
#import "TiUtils.h"

@implementation ComGooglecodeQuicktigame2dBox2dRevJointProxy

-(void)setMotorSpeed:(id)args
{
    [lock lock];
    
    ENSURE_SINGLE_ARG(args,NSNumber);
    
    CGFloat speed = [TiUtils floatValue:args];
    ((b2RevoluteJoint*)joint)->SetMotorSpeed(speed);
    
    [lock unlock];
}

-(void)setMaxMotorTorque:(id)args
{
    [lock lock];
    
    ENSURE_SINGLE_ARG(args,NSNumber);
    
    CGFloat t = [TiUtils floatValue:args];
    ((b2RevoluteJoint*)joint)->SetMaxMotorTorque(t);
    
    [lock unlock];
}

-(id)getJointAngle:(id)args
{
    float angle = ((b2RevoluteJoint*)joint)->GetJointAngle();
    return NUMFLOAT(angle);
}

-(id)getJointSpeed:(id)args
{
    float speed = ((b2RevoluteJoint*)joint)->GetJointSpeed();
    return NUMFLOAT(speed);
}



@end
