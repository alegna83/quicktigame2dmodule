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
#import "TiProxy.h"
#import <Box2D/Box2D.h>

@interface ComGooglecodeQuicktigame2dBox2dMouseJointProxy : TiProxy {
	b2MouseJoint *joint;
    NSRecursiveLock *lock;
    CGFloat height;
    
}
-(id)initWithJoint:(b2MouseJoint*)joint;
-(id)initWithJointAndHeight:(b2MouseJoint*)joint,CGFloat height;
-(b2Joint*)joint;
@end
