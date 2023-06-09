﻿/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/ 

using UnityEngine;

namespace Unity.Robotics.UrdfImporter
{
    public class UrdfJointFixed : UrdfJoint
    {
        public override JointTypes JointType => JointTypes.Fixed;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointFixed urdfJoint = linkObject.AddComponent<UrdfJointFixed>();
            #if UNITY_2020_1_OR_NEWER
                urdfJoint.unityJoint = linkObject.GetComponent<ArticulationBody>();
            #else
                        urdfJoint.UnityJoint = linkObject.AddComponent<FixedJoint>();
                        urdfJoint.UnityJoint.autoConfigureConnectedAnchor = true;
            #endif

            return urdfJoint;
        }

        protected override bool IsJointAxisDefined()
        {
            return true; //Axis isn't used
        }
    }
}

