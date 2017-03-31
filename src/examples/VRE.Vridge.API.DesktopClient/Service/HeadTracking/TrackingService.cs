using System;
using System.Windows.Media.Media3D;
using System.Runtime.InteropServices;
using VRE.Vridge.API.Client.Helpers;
using VRE.Vridge.API.Client.Proxy.HeadTracking;

namespace VRE.Vridge.API.DesktopTester.Service.PSMovePose
{
}

namespace VRE.Vridge.API.DesktopTester.Service.HeadTracking
{
    public class PSMoveServiceClientAPI
    {
        public enum PSMResult
        {
            PSMResult_Error,
            PSMResult_Success,
            PSMResult_Timeout,
            PSMResult_RequestSent,
            PSMResult_Canceled,
            PSMResult_NoData,
        };

        public enum PSMControllerType
        {
            PSMController_None = -1,
            PSMController_Move,
            PSMController_Navi,
            PSMController_DualShock4
        }

        public enum PSMConnectionType
        {
            PSMConnectionType_BLUETOOTH,
            PSMConnectionType_USB
        }

        public enum PSMTrackingColorType
        {
            PSMTrackingColorType_Magenta,    // R:0xFF, G:0x00, B:0xFF
            PSMTrackingColorType_Cyan,       // R:0x00, G:0xFF, B:0xFF
            PSMTrackingColorType_Yellow,     // R:0xFF, G:0xFF, B:0x00
            PSMTrackingColorType_Red,        // R:0xFF, G:0x00, B:0x00
            PSMTrackingColorType_Green,      // R:0x00, G:0xFF, B:0x00
            PSMTrackingColorType_Blue,       // R:0x00, G:0x00, B:0xFF

            PSMTrackingColorType_MaxColorTypes
        }

        /// A 2D vector with float components.
        public struct PSMVector2f
        {
            float x, y;
        }

        /// A 3D vector with float components.
        public struct PSMVector3f
        {
            float x, y, z;
        }

        /// A 3D vector with int components.
        public struct PSMVector3i
        {
            int x, y, z;
        }
        /*
        public struct PSMMatrix3f
        {
            float m[3][3]; // storage is row major order: [x0,x1,x2,y0,y1,y1,z0,z1,z2]
        }
        */
        /// A quaternion rotation.
        public struct PSMQuatf
        {
            float w, x, y, z;
        }

        /// Position and orientation together.
        public struct PSMPosef
        {
            PSMVector3f Position;
            PSMQuatf Orientation;
        }

        public struct PSMPSMove
        {
            bool bHasValidHardwareCalibration;
            bool bIsTrackingEnabled;
            bool bIsCurrentlyTracking;
            bool bIsOrientationValid;
            bool bIsPositionValid;
            bool bHasUnpublishedState;

            char[] DevicePath;
            char[] DeviceSerial;
            char[] AssignedHostSerial;
            bool PairedToHost;
            PSMConnectionType ConnectionType;

            PSMTrackingColorType TrackingColorType;
            PSMPosef Pose; /*
        PSMPhysicsData PhysicsData;
        PSMPSMoveRawSensorData RawSensorData;
        PSMPSMoveCalibratedSensorData CalibratedSensorData;
        PSMRawTrackerData RawTrackerData;

        PSMButtonState TriangleButton;
        PSMButtonState CircleButton;
        PSMButtonState CrossButton;
        PSMButtonState SquareButton;
        PSMButtonState SelectButton;
        PSMButtonState StartButton;
        PSMButtonState PSButton;
        PSMButtonState MoveButton;
        PSMButtonState TriggerButton;
        PSMBatteryState BatteryValue;
        unsigned char TriggerValue;
        unsigned char Rumble;
        unsigned char LED_r, LED_g, LED_b;

        long long ResetPoseButtonPressTime;
        bool bResetPoseRequestSent;
        bool bPoseResetButtonEnabled;
        */
        }

        public struct PSMController
        {
            int ControllerID;
            PSMControllerType ControllerType;
            /*
            [StructLayout(LayoutKind.Explicit)]
            public struct ControllerState
            {
                PSMPSMove PSMoveState;
                PSMPSNavi PSNaviState;
                PSMDualShock4 PSDS4State;
            }*/
            PSMPSMove PSMoveState;
            bool bValid;
            int OutputSequenceNum;
            int InputSequenceNum;
            bool IsConnected;
            ulong DataFrameLastReceivedTime;
            float DataFrameAverageFPS;
            int ListenerCount;
        }


        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern PSMResult PSM_InitializeAsync(
            [MarshalAs(UnmanagedType.LPStr)]String host,    // "localhost"
            [MarshalAs(UnmanagedType.LPStr)]String port);   // "9512"

        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern PSMResult PSM_Shutdown();

        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern PSMResult PSM_Update();

        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern String PSM_GetClientVersionString();
        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern bool PSM_GetIsInitialized();
        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern bool PSM_GetIsConnected();
        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern bool PSM_HasConnectionStatusChanged();
        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern bool PSM_HasControllerListChanged();
        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern bool PSM_HasTrackerListChanged();
        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern bool PSM_HasHMDListChanged();
        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern bool PSM_WasSystemButtonPressed();

        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern PSMController PSM_GetController(int controller_id);

        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern PSMResult PSM_GetControllerOrientation(
            int controller_id, [MarshalAs(UnmanagedType.LPStr)]PSMQuatf out_orientation);
        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern PSMResult PSM_GetControllerPosition(
            int controller_id, [MarshalAs(UnmanagedType.LPStr)]PSMVector3f out_position);
        [DllImport("PSMoveClient_CAPI.dll")]
        public static extern PSMResult PSM_GetControllerPose(
            int controller_id, [MarshalAs(UnmanagedType.LPStr)]PSMPosef out_pose);


    }

    public class TrackingService
    {               
        private readonly HeadTrackingProxy proxy;

        private Vector3D position;
        private Matrix3D offsetMatrix;

        public TrackingService(HeadTrackingProxy proxy)
        {
            this.proxy = proxy;
        }

        /// <summary>
        /// Sends absolute position to VR without touching rotation.
        /// </summary>        
        public void SendPositionOnly(double x, double y, double z)
        {
            position = new Vector3D(x, y, z);
            proxy.SetPosition((float)x, (float)y, (float)z);
        }
               
        /// <summary>
        /// Sends absolute position and rotation. VRidge will expect a steady stream of rotational data
        /// after using this call. It will timeout and revert back to mobile tracking data if no rotational data 
        /// is sent for a while.
        /// </summary>        
        public void SendRotationAndPosition(double yaw, double pitch, double roll, double x, double y, double z)
        {
            position = new Vector3D(x, y, z);

            // Invert yaw so sliding to the right rotates to the right
            // Makes more sense with horizontal slider

            proxy.SetRotationAndPosition(
                (float)MathHelpers.DegToRad(-yaw),
                (float)MathHelpers.DegToRad(pitch),
                (float)MathHelpers.DegToRad(roll),
                (float)x, (float)y, (float)z);
        }

        /// <summary>
        /// Sends absolute position and rotation as a quaternion. VRidge will expect a steady stream of rotational data
        /// after using this call. It will timeout and revert back to mobile tracking data if no rotational data 
        /// is sent for a while.
        /// </summary>        
        public void SendQuatRotationAndPosition(double qw, double qx, double qy, double qz, double x, double y, double z)
        {
            position = new Vector3D(x, y, z);

            // Invert yaw so sliding to the right rotates to the right
            // Makes more sense with horizontal slider

            proxy.SetQuatRotationAndPosition(
                (float)qw,
                (float)qx,
                (float)qy,
                (float)qz,
                (float)x, (float)y, (float)z);
        }

        /// <summary>
        /// Return latest mobile pose. Can be used by async offset user to find
        /// a reference point for drift correction.
        /// </summary>
        /// <returns>Column-major 4x4 pose matrix.</returns>
        public float[] GetCurrentPhonepose()
        {
            return proxy.GetCurrentPhonePose();
        }

        /// <summary>
        /// Store rotational offset matrix on VRidge side which will be combined with 
        /// every incoming mobile pose matrix. Effectively: lagless offset.
        /// </summary>
        public bool SendAsyncOffset(double yaw, double pitch, double roll)
        {
            // Invert yaw so sliding to the right rotates to the right
            // Makes more sense with horizontal slider

            return proxy.SetAsyncOffset(
                (float)MathHelpers.DegToRad(-yaw),
                (float)MathHelpers.DegToRad(pitch),
                (float)MathHelpers.DegToRad(roll));
        }

        /// <summary>
        /// Resets offset stored with <see cref="SendAsyncOffset"/>
        /// </summary>        
        public bool ResetAsyncOffset()
        {
            if (proxy == null) return false;

            return proxy.ResetAsyncOffset();
        }

        /// <summary>
        /// Begin listening to mobile tracking data. <see cref="OnNewSyncPose"/>
        /// is called whenever new data is available and modifiable.
        /// </summary>
        public void BeginSyncOffsetMode()
        {
            if (proxy == null) return;

            proxy.BeginSyncOffset(OnNewSyncPose);
            proxy.SyncModeDisconnected += OnSyncDisconnect;
        }

        /// <summary>
        /// Update local matrix that is combined with each mobile matrix for sync offset mode.
        /// </summary> 
        public void UpdateOffsetMatrix(double yaw, double pitch, double roll, double x, double y, double z)
        {
            // Create 4x4 transformation matrix with help of WPF built-in methods
            offsetMatrix = Matrix3D.Identity;

            position = new Vector3D(x, y, z);

            offsetMatrix.Rotate(Helpers.QuaternionFromYawPitchRoll((float)MathHelpers.DegToRad(yaw), (float)MathHelpers.DegToRad(pitch), (float)MathHelpers.DegToRad(roll)));

            //offsetMatrix.Rotate(new Quaternion(s * pitch, s * yaw, s * roll, c));
        }

        /// <summary>
        /// Stops listening to mobile tracking data.
        /// </summary>
        public void StopSyncOffsetMode()
        {
            if (proxy == null) return;

            proxy.StopSyncOffset();
            proxy.SyncModeDisconnected -= OnSyncDisconnect;
        }

        /// <summary>
        /// Close the connection and let other API clients use head tracking service.
        /// </summary>
        public void Disconnect()
        {
            proxy?.Disconnect();
        }

        /// <summary>
        /// Called whenever new mobile pose is sent to VRidge.
        /// </summary>        
        private void OnNewSyncPose(float[] poseMatrix)
        {
            /* Android's Matrix is column-major while .NET's Matrix3D uses row-major layout 
             * therefore matrix transposition is required */
            Matrix3D currentData = new Matrix3D(
                poseMatrix[0], poseMatrix[4], poseMatrix[8], poseMatrix[12],
                poseMatrix[1], poseMatrix[5], poseMatrix[9], poseMatrix[13],
                poseMatrix[2], poseMatrix[6], poseMatrix[10], poseMatrix[14],
                poseMatrix[3], poseMatrix[8], poseMatrix[11], poseMatrix[15]
                );

            // Now we combine our offset with phone pose (only rotations are used by both matrices)
            var combinedMatrix = Matrix3D.Multiply(currentData, offsetMatrix);

            // Override position with absolutely placed position
            combinedMatrix.OffsetX = position.X;
            combinedMatrix.OffsetY = position.Y;
            combinedMatrix.OffsetZ = position.Z;                        

            // Override original matrix data with our modified matrix
            Array.Copy(combinedMatrix.FlattenAsColumnMajor(), poseMatrix, 16);

            // Arrays are passed by reference so calling method will have modified data
        }

        private void OnSyncDisconnect(object sender, Exception e)
        {
            StopSyncOffsetMode();
        }
    }
}
