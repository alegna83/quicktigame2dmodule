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

#include <Box2D/Box2D.h>

class ComGooglecodeQuicktigame2dWorldQueryCallback : public b2QueryCallback
{
public:
    ComGooglecodeQuicktigame2dWorldQueryCallback(const b2Vec2& point)
    {
        m_point = point;
        m_fixture = NULL;
    }    
    bool ReportFixture(b2Fixture* fixture)
    {
        b2Body* body = fixture->GetBody();
        if (body->GetType() == b2_dynamicBody || body->GetType() == b2_kinematicBody)
        {
            bool inside = fixture->TestPoint(m_point);
            if (inside)
            {
                m_fixture = fixture;                
                // We are done, terminate the query.
                return false;
            }
        }        
        // Continue the query.
        return true;
    }
    b2Vec2 m_point;
    b2Fixture* m_fixture;
};
