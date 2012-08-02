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
#import "ComGooglecodeQuicktigame2dBox2dJointProxy.hh"
#import "ComGooglecodeQuicktigame2dGameView.h"
#import "ComGooglecodeQuicktigame2dBox2dWorldProxy.hh"
#import <Box2D/Box2D.h>

#import "TiUtils.h"

@implementation ComGooglecodeQuicktigame2dBox2dJointProxy

-(id)initWithJoint:(b2Joint*)joint_
{
    self = [super init];
    if (self != nil) {
		joint = joint_;
        lock = [[NSRecursiveLock alloc] init];
    }
    return self;    
}
-(id)initWithJointAndHeight:(b2Joint*)joint_,CGFloat height_
{
    self = [super init];
    if (self != nil) {
		joint = joint_;
        lock = [[NSRecursiveLock alloc] init];
        height = height_;
    }
    return self;    
}

-(void)dealloc
{
    RELEASE_TO_NIL(lock);
	[super dealloc];
}

-(b2Joint*)joint
{
    return joint;
}


@end
