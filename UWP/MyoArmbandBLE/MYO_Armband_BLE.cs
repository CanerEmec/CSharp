using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.Advertisement;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Devices.Enumeration;
using Windows.Devices.Bluetooth.Rfcomm;
using System.Runtime.InteropServices;
using Windows.Storage.Streams;

namespace BIONIC_ARM
{
    class MYO_Armband_BLE
    {
        // once baglantı yapılıp service ve characteristicler alınmalı ...
        public static List<GattCharacteristic> gattCharacteristics = new List<GattCharacteristic>();
        public static List<GattDeviceService> gattDeviceServices = new List<GattDeviceService>();

        public static GattCharacteristic GetGattCharacteristic(string name)
        {
            foreach (GattCharacteristic item in gattCharacteristics)
            {
                if(item.Uuid == myohw_services_GUID[name])
                {
                    return item;
                }
            }
            return null;            
        } 

        public static GattDeviceService GetGattDeviceService(string name)
        {
            foreach (GattDeviceService item in gattDeviceServices)
            {
                if (item.Uuid == myohw_services_GUID[name])
                {
                    return item;
                }
            }
            return null;
        }

        #region ReadGattCharacteristic method overloading

        public static object ReadGattCharacteristic(string characteristic_name)
        {
            GattCharacteristic chr = GetGattCharacteristic(characteristic_name);
            GattCharacteristicProperties property = chr.CharacteristicProperties;
            if (property.HasFlag(GattCharacteristicProperties.Read))
            {
                GattReadResult result = chr.ReadValueAsync().AsTask().ConfigureAwait(false).GetAwaiter().GetResult();
                if (result.Status == GattCommunicationStatus.Success)
                {
                    var reader = DataReader.FromBuffer(result.Value);
                    return reader;
                }
            }
            return null;
        }

        public static object ReadGattCharacteristic(myohw_services characteristic_name)
        {
            return ReadGattCharacteristic(characteristic_name.ToString());
        }

        public static object ReadGattCharacteristic(myohw_standard_services characteristic_name)
        {
            return ReadGattCharacteristic(characteristic_name.ToString());
        }

        #endregion

        #region WriteGattCharacteristic method overloadings..

        public static void WriteGattCharacteristic(string characteristic_name,byte[] dataToWrite)
        {
            GattCharacteristic chr = GetGattCharacteristic(characteristic_name);
            GattCharacteristicProperties property = chr.CharacteristicProperties;
            if (property.HasFlag(GattCharacteristicProperties.Write))
            {
                var writer = new DataWriter();
                writer.WriteBytes(dataToWrite);
                
                GattCommunicationStatus result = chr.WriteValueAsync(writer.DetachBuffer()).AsTask().ConfigureAwait(false).GetAwaiter().GetResult();
                if (result == GattCommunicationStatus.Success)
                {
                    //Do stg. if success
                }
            }
        }

        public static void WriteGattCharacteristic(myohw_standard_services characteristic_name, byte[] dataToWrite)
        {
            string chr_name = characteristic_name.ToString();
            WriteGattCharacteristic(chr_name, dataToWrite);
        }

        public static void WriteGattCharacteristic(myohw_services characteristic_name, byte[] dataToWrite)
        {
            string chr_name = characteristic_name.ToString();
            WriteGattCharacteristic(chr_name, dataToWrite);
        }

        public static async void WriteGattCharacteristic(string characteristic_name, string dataToWrite)
        {
            GattCharacteristic chr = GetGattCharacteristic(characteristic_name);
            GattCharacteristicProperties property = chr.CharacteristicProperties;
            if (property.HasFlag(GattCharacteristicProperties.Write))
            {
                var writer = new DataWriter();

                writer.WriteString(dataToWrite);
                GattCommunicationStatus result = await chr.WriteValueAsync(writer.DetachBuffer());
                if (result == GattCommunicationStatus.Success)
                {
                    //Do stg. if success
                }
            }
        }

        #endregion

        #region ByteArrayToStruct method overloading

        public static myohw_emg_data_t ByteArrayToStruct(byte[] bytes)
        {
            myohw_emg_data_t _struct = new myohw_emg_data_t();
            int size = Marshal.SizeOf(_struct);

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.Copy(bytes, 0, ptr, size);
            _struct = (myohw_emg_data_t)Marshal.PtrToStructure(ptr, _struct.GetType());
            Marshal.FreeHGlobal(ptr);

            return _struct;
        }

        public static object ByteArrayToStruct(byte[] bytes,DataTypes dataType)
        {
            int size;
            IntPtr ptr;
            switch (dataType)
            {
                case DataTypes.Imu:
                    var ImuStruct = new myohw_imu_data_t();
                    size = Marshal.SizeOf(ImuStruct);

                    ptr = Marshal.AllocHGlobal(size);
                    Marshal.Copy(bytes, 0, ptr, size);
                    ImuStruct = (myohw_imu_data_t)Marshal.PtrToStructure(ptr, ImuStruct.GetType());
                    Marshal.FreeHGlobal(ptr);

                    return ImuStruct;
                case DataTypes.Emg:
                    var EmgStruct = new myohw_emg_data_t();
                    size = Marshal.SizeOf(EmgStruct);

                    ptr = Marshal.AllocHGlobal(size);
                    Marshal.Copy(bytes, 0, ptr, size);
                    EmgStruct = (myohw_emg_data_t)Marshal.PtrToStructure(ptr, EmgStruct.GetType());
                    Marshal.FreeHGlobal(ptr);

                    return EmgStruct;
            }
            return null;
        }

        #endregion

        #region StructToByteArray method overloading...

        public static byte[] StructToByteArray(myohw_command_set_mode_t _struct)
        {
            int size = Marshal.SizeOf(_struct);
            byte[] bytes = new byte[size];

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(_struct, ptr, true);
            Marshal.Copy(ptr, bytes, 0, size);
            Marshal.FreeHGlobal(ptr);
            return bytes;
        }

        public static byte[] StructToByteArray(myohw_command_vibrate_t _struct)
        {
            int size = Marshal.SizeOf(_struct);
            byte[] bytes = new byte[size];

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(_struct, ptr, true);
            Marshal.Copy(ptr, bytes, 0, size);
            Marshal.FreeHGlobal(ptr);
            return bytes;
        }

        public static byte[] StructToByteArray(myohw_command_deep_sleep_t _struct)
        {
            int size = Marshal.SizeOf(_struct);
            byte[] bytes = new byte[size];

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(_struct, ptr, true);
            Marshal.Copy(ptr, bytes, 0, size);
            Marshal.FreeHGlobal(ptr);
            return bytes;
        }

        public static byte[] StructToByteArray(myohw_command_set_sleep_mode_t _struct)
        {
            int size = Marshal.SizeOf(_struct);
            byte[] bytes = new byte[size];

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(_struct, ptr, true);
            Marshal.Copy(ptr, bytes, 0, size);
            Marshal.FreeHGlobal(ptr);
            return bytes;
        }

        public static byte[] StructToByteArray(myohw_command_unlock_t _struct)
        {
            int size = Marshal.SizeOf(_struct);
            byte[] bytes = new byte[size];

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(_struct, ptr, true);
            Marshal.Copy(ptr, bytes, 0, size);
            Marshal.FreeHGlobal(ptr);
            return bytes;
        }

        public static byte[] StructToByteArray(myohw_command_user_action_t _struct)
        {
            int size = Marshal.SizeOf(_struct);
            byte[] bytes = new byte[size];

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(_struct, ptr, true);
            Marshal.Copy(ptr, bytes, 0, size);
            Marshal.FreeHGlobal(ptr);
            return bytes;
        }
        #endregion

        #region HELPER

        public static myohw_command_set_mode_t GetFormatFor_myoCommandSetMode(myohw_emg_mode_t emg_mode, myohw_imu_mode_t imu_mode, myohw_classifier_mode_t classifier_mode)
        {
            myohw_command_set_mode_t result;
            result.emg_mode = (byte)emg_mode;
            result.imu_mode = (byte)imu_mode;
            result.classifier_mode = (byte)classifier_mode;
            result.header = new myohw_command_header_t()
            {
                payload_size = (byte)0x03,
                command = (byte)myohw_command_t.myohw_command_set_mode
            };
            return result;
            
        }

        public static myohw_command_vibrate_t GetFormatFor_myoCommandVibrate(myohw_vibration_type_t vibration_type)
        {
            myohw_command_vibrate_t result;
            result.type = (byte)vibration_type;
            result.header = new myohw_command_header_t()
            {
                command = (byte)myohw_command_t.myohw_command_vibrate,
                payload_size = (byte)0x01,
            };
            return result;

        }

        public static myohw_command_deep_sleep_t GetFormatFor_myoCommandDeepSlep()
        {
            myohw_command_deep_sleep_t result;
            result.header = new myohw_command_header_t
            {
                payload_size = (byte)0x00,
                command = (byte)myohw_command_t.myohw_command_deep_sleep,
            };
            return result;
        }

        public static myohw_command_set_sleep_mode_t GetFormatFor_myoSleepMode(myohw_sleep_mode_t sleepMode)
        {
            myohw_command_set_sleep_mode_t result;
            result.sleep_mode = (byte)sleepMode;
            result.header = new myohw_command_header_t
            {
                command = (byte)myohw_command_t.myohw_command_set_sleep_mode,
                payload_size = (byte)0x01,
            };
            return result;
        }

        public static myohw_command_unlock_t GetFormatFor_myoUnlock(myohw_unlock_type_t unlockType)
        {
            myohw_command_unlock_t result;
            result.type = (Byte)unlockType;
            result.header = new myohw_command_header_t
            {
                command = (Byte)myohw_command_t.myohw_command_unlock,
                payload_size = (Byte)0x01
            };
            return result;
        }

        public static myohw_command_user_action_t GetFormatFor_myoUserAction(myohw_user_action_type_t userAction)
        {
            myohw_command_user_action_t result;
            result.type = (Byte)userAction;
            result.header = new myohw_command_header_t
            {
                command = (Byte)myohw_command_t.myohw_command_user_action,
                payload_size = (Byte)0x01,
            };
            return result;
        }

        public enum DataTypes
        {
            Imu,
            Emg
        }

        #endregion

        #region COSTANTS
        public static Dictionary<string,Guid> myohw_services_GUID { get
            {
                return new Dictionary<string, Guid>()
                {
                    { "ControlService",new Guid("d5060001-a904-deb9-4748-2c7f4a124842") },
                    { "MyoInfoCharacteristic",new Guid("d5060101-a904-deb9-4748-2c7f4a124842") },

                    { "FirmwareVersionCharacteristic",new Guid("d5060201-a904-deb9-4748-2c7f4a124842") },

                    { "CommandCharacteristic",new Guid("d5060401-a904-deb9-4748-2c7f4a124842") },

                    { "ImuDataService",new Guid("d5060002-a904-deb9-4748-2c7f4a124842") },
                    { "IMUDataCharacteristic",new Guid("d5060402-a904-deb9-4748-2c7f4a124842") },
                    { "MotionEventCharacteristic",new Guid("d5060502-a904-deb9-4748-2c7f4a124842") },

                    { "ClassifierService",new Guid("d5060003-a904-deb9-4748-2c7f4a124842") },
                    { "ClassifierEventCharacteristic",new Guid("d5060103-a904-deb9-4748-2c7f4a124842") },

                    { "EmgDataService",new Guid("d5060005-a904-deb9-4748-2c7f4a124842") },
                    { "EmgData0Characteristic",new Guid("d5060105-a904-deb9-4748-2c7f4a124842") },
                    { "EmgData1Characteristic",new Guid("d5060205-a904-deb9-4748-2c7f4a124842") },
                    { "EmgData2Characteristic",new Guid("d5060305-a904-deb9-4748-2c7f4a124842") },
                    { "EmgData3Characteristic",new Guid("d5060405-a904-deb9-4748-2c7f4a124842") },

                    { "BatteryService",new Guid("0000180f-0000-1000-8000-00805f9b34fb") },
                    { "BatteryLevelCharacteristic",new Guid("00002a19-0000-1000-8000-00805f9b34fb") },
                    { "DeviceName",new Guid("00002a00-0000-1000-8000-00805f9b34fb") },

                };
            }
        }


        public enum myohw_services
        {
            ControlService = (UInt16)0x0001,                    ////< Myo info service
            MyoInfoCharacteristic = (UInt16)0x0101,             ////< Serial number for this Myo and various parameters which
                                                                ////< are specific to this firmware. Read-only attribute. 
                                                                ////< See myohw_fw_info_t.
            FirmwareVersionCharacteristic = (UInt16)0x0201,     ////< Current firmware version. Read-only characteristic.
                                                                ////< See myohw_fw_version_t.
            CommandCharacteristic = (UInt16)0x0401,             ////< Issue commands to the Myo. Write-only characteristic.
                                                                ////< See myohw_command_t.

            ImuDataService = (UInt16)0x0002,                    ////< IMU service
            IMUDataCharacteristic = (UInt16)0x0402,             ////< See myohw_imu_data_t. Notify-only characteristic.
            MotionEventCharacteristic = (UInt16)0x0502,         ////< Motion event data. Indicate-only characteristic.

            ClassifierService = (UInt16)0x0003,                 ////< Classifier event service.
            ClassifierEventCharacteristic = (UInt16)0x0103,     ////< Classifier event data. Indicate-only characteristic. See myohw_pose_t.

            EmgDataService = (UInt16)0x0005,                    ////< Raw EMG data service.
            EmgData0Characteristic = (UInt16)0x0105,            ////< Raw EMG data. Notify-only characteristic.
            EmgData1Characteristic = (UInt16)0x0205,            ////< Raw EMG data. Notify-only characteristic.
            EmgData2Characteristic = (UInt16)0x0305,            ////< Raw EMG data. Notify-only characteristic.
            EmgData3Characteristic = (UInt16)0x0405,            ////< Raw EMG data. Notify-only characteristic.
        }

        // Standard Bluetooth device services.
        public enum myohw_standard_services
        {
            BatteryService = (UInt16)0x180f,                    ////< Battery service
            BatteryLevelCharacteristic = (UInt16)0x2a19,        ////< Current battery level information. Read/notify characteristic.
            DeviceName = (UInt16)0x2a00,                        ////< Device name data. Read/write characteristic.
        }

        /// Supported poses.
        public enum myohw_pose_t
        {
            myohw_pose_rest = (UInt16)0x0000,
            myohw_pose_fist = (UInt16)0x0001,
            myohw_pose_wave_in = (UInt16)0x0002,
            myohw_pose_wave_out = (UInt16)0x0003,
            myohw_pose_fingers_spread = (UInt16)0x0004,
            myohw_pose_double_tap = (UInt16)0x0005,
            myohw_pose_unknown = (UInt16)0xffff
        }

        /// Various parameters that may affect the behaviour of this Myo armband.
        /// The Myo library reads this attribute when a connection is established.
        /// Value layout for the myohw_att_handle_fw_info attribute.
        ///[Serializable]
        public struct myohw_fw_info_t
        {
            public Byte[] serial_number; //[6]   ////< Unique serial number of this Myo.
            public UInt16 unlock_pose;           ////< Pose that should be interpreted as the unlock pose. See myohw_pose_t.
            public Byte active_classifier_type;  ////< Whether Myo is currently using a built-in or a custom classifier.
                                                 ////< See myohw_classifier_model_type_t.
            public Byte active_classifier_index; ////< Index of the classifier that is currently active.
            public Byte has_custom_classifier;   ////< Whether Myo contains a valid custom classifier. 1 if it does, otherwise 0.
            public Byte stream_indicating;       ////< Set if the Myo uses BLE indicates to stream data, for reliable capture.
            public Byte sku;                     ////< SKU value of the device. See myohw_sku_t
            public Byte[] reserved;  //[7]       ////< Reserved for future use; populated with zeros.
        }

        // Known Myo SKUs
        public enum myohw_sku_t
        {
            myohw_sku_unknown = (Byte)0,          ////< Unknown SKU (default value for old firmwares)
            myohw_sku_black_myo = (Byte)1,        ////< Black Myo
            myohw_sku_white_myo = (Byte)2         ////< White Myo
        }


        /// Known Myo hardware revisions.
        public enum myohw_hardware_rev_t
        {
            myohw_hardware_rev_unknown = (Byte)0, ////< Unknown hardware revision.
            myohw_hardware_rev_revc = (Byte)1,    ////< Myo Alpha (REV-C) hardware.
            myohw_hardware_rev_revd = (Byte)2,    ////< Myo (REV-D) hardware.
            myohw_num_hardware_revs               ////< Number of hardware revisions known; not a valid hardware revision.
        }


        /// Version information for the Myo firmware.
        /// Value layout for the myohw_att_handle_fw_version attribute.
        /// Minor version is incremented for changes in this interface.
        /// Patch version is incremented for firmware changes that do not introduce changes in this interface.
        public struct myohw_fw_version_t
        {
            public UInt16 major;
            public UInt16 minor;
            public UInt16 patch;
            public UInt16 hardware_rev;            ////< Myo hardware revision. See myohw_hardware_rev_t.
        }

        /// Kinds of commands.
        public enum myohw_command_t
        {
            myohw_command_set_mode = (Byte)0x01,          ////< Set EMG and IMU modes. See myohw_command_set_mode_t.
            myohw_command_vibrate = (Byte)0x03,           ////< Vibrate. See myohw_command_vibrate_t.
            myohw_command_deep_sleep = (Byte)0x04,        ////< Put Myo into deep sleep. See myohw_command_deep_sleep_t.
            myohw_command_vibrate2 = (Byte)0x07,          ////< Extended vibrate. See myohw_command_vibrate2_t.
            myohw_command_set_sleep_mode = (Byte)0x09,    ////< Set sleep mode. See myohw_command_set_sleep_mode_t.
            myohw_command_unlock = (Byte)0x0a,            ////< Unlock Myo. See myohw_command_unlock_t.
            myohw_command_user_action = (Byte)0x0b,       ////< Notify user that an action has been recognized / confirmed.
                                                          ////< See myohw_command_user_action_t.
        }


        /// Header that every command begins with.
        public struct myohw_command_header_t
        {
            public Byte command;                           ////< Command to send. See myohw_command_t.
            public Byte payload_size;                      ////< Number of bytes in payload.
        }

        /// EMG modes.
        public enum myohw_emg_mode_t
        {
            myohw_emg_mode_none = (Byte)0x00,             ////< Do not send EMG data.
            myohw_emg_mode_send_emg = (Byte)0x02,         ////< Send filtered EMG data.
            myohw_emg_mode_send_emg_raw = (Byte)0x03,     ////< Send raw (unfiltered) EMG data.
        }


        /// IMU modes.
        public enum myohw_imu_mode_t
        {
            myohw_imu_mode_none = (Byte)0x00,             ////< Do not send IMU data or events.
            myohw_imu_mode_send_data = (Byte)0x01,        ////< Send IMU data streams (accelerometer, gyroscope, and orientation).
            myohw_imu_mode_send_events = (Byte)0x02,      ////< Send motion events detected by the IMU (e.g. taps).
            myohw_imu_mode_send_all = (Byte)0x03,         ////< Send both IMU data streams and motion events.
            myohw_imu_mode_send_raw = (Byte)0x04,         ////< Send raw IMU data streams.
        }


        /// Classifier modes.
        public enum myohw_classifier_mode_t
        {
            myohw_classifier_mode_disabled = (Byte)0x00,  ////< Disable and reset the internal state of the onboard classifier.
            myohw_classifier_mode_enabled = (Byte)0x01,   ////< Send classifier events (poses and arm events).
        }


        /// Command to set EMG and IMU modes.
        public struct myohw_command_set_mode_t
        {
            public myohw_command_header_t header;          ////< command == myohw_command_set_mode. payload_size = 3.
            public Byte emg_mode;                          ////< EMG sensor mode. See myohw_emg_mode_t.
            public Byte imu_mode;                          ////< IMU mode. See myohw_imu_mode_t.
            public Byte classifier_mode;                   ////< Classifier mode. See myohw_classifier_mode_t.
        }

        /// Kinds of vibrations.
        public enum myohw_vibration_type_t
        {
            myohw_vibration_none = (Byte)0x00,            ////< Do not vibrate.
            myohw_vibration_short = (Byte)0x01,           ////< Vibrate for a short amount of time.
            myohw_vibration_medium = (Byte)0x02,          ////< Vibrate for a medium amount of time.
            myohw_vibration_long = (Byte)0x03,            ////< Vibrate for a long amount of time.
        }


        /// Vibration command.
        public struct myohw_command_vibrate_t
        {
            public myohw_command_header_t header;          ////< command == myohw_command_vibrate. payload_size == 1.
            public Byte type;                              ////< See myohw_vibration_type_t.
        }


        /// Deep sleep command.
        public struct myohw_command_deep_sleep_t
        {
            public myohw_command_header_t header;          ////< command == myohw_command_deep_sleep. payload_size == 0.
        }


        /// Sleep modes.
        public enum myohw_sleep_mode_t
        {
            myohw_sleep_mode_normal = (Byte)0,            ////< Normal sleep mode; Myo will sleep after a period of inactivity.
            myohw_sleep_mode_never_sleep = (Byte)1,       ////< Never go to sleep.
        }


        /// Set sleep mode command.
        public struct myohw_command_set_sleep_mode_t
        {
            public myohw_command_header_t header;          ////< command == myohw_command_set_sleep_mode. payload_size == 1.
            public Byte sleep_mode;                        ////< Sleep mode. See myohw_sleep_mode_t.
        }

        /// Unlock types.
        public enum myohw_unlock_type_t
        {
            myohw_unlock_lock = (Byte)0x00,                 ////< Re-lock immediately.
            myohw_unlock_timed = (Byte)0x01,                ////< Unlock now and re-lock after a fixed timeout.
            myohw_unlock_hold = (Byte)0x02,                 ////< Unlock now and remain unlocked until a lock command is received.
        }


        /// Unlock Myo command.
        /// Can also be used to force Myo to re-lock.
        public struct myohw_command_unlock_t
        {
            public myohw_command_header_t header;           ////< command == myohw_command_unlock. payload_size == 1.
            public Byte type;                               ////< Unlock type. See myohw_unlock_type_t.
        }


        /// User action types.
        public enum myohw_user_action_type_t
        {
            myohw_user_action_single = (Byte)0,             ////< User did a single, discrete action, such as pausing a video.
        }


        /// User action command.
        public struct myohw_command_user_action_t
        {
            public myohw_command_header_t header;           ////< command == myohw_command_user_action. payload_size == 1.
            public Byte type;                               ////< Type of user action that occurred. See myohw_user_action_type_t.
        }

        /// Classifier model types
        public enum myohw_classifier_model_type_t
        {
            myohw_classifier_model_builtin = (Byte)0,        ////< Model built into the classifier package.
            myohw_classifier_model_custom = (Byte)1          ////< Model based on personalized user data.
        }

        /// Integrated motion data.
        public struct myohw_imu_data_t
        {
            /// Orientation data, represented as a unit quaternion. Values are multiplied by MYOHW_ORIENTATION_SCALE.
            /*public struct orientation
            {
                public Int16 w, x, y, z;
            }*/
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public Int16[] orientation;//4

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public Int16[] accelerometer;//3                    ////< Accelerometer data. In units of g. Range of + -16.
                                                                ////< Values are multiplied by MYOHW_ACCELEROMETER_SCALE.
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public Int16[] gyroscope;//3                        ////< Gyroscope data. In units of deg/s. Range of + -2000.
                                                                ////< Values are multiplied by MYOHW_GYROSCOPE_SCALE.
        }


        /// Types of classifier events.
        public enum myohw_classifier_event_type_t
        {
            myohw_classifier_event_arm_synced = (Byte)0x01,
            myohw_classifier_event_arm_unsynced = (Byte)0x02,
            myohw_classifier_event_pose = (Byte)0x03,
            myohw_classifier_event_unlocked = (Byte)0x04,
            myohw_classifier_event_locked = (Byte)0x05,
            myohw_classifier_event_sync_failed = (Byte)0x06,
        }


        /// Enumeration identifying a right arm or left arm.
        public enum myohw_arm_t
        {
            myohw_arm_right = (Byte)0x01,
            myohw_arm_left = (Byte)0x02,
            myohw_arm_unknown = (Byte)0xff
        }


        /// Possible directions for Myo's +x axis relative to a user's arm.
        public enum myohw_x_direction_t
        {
            myohw_x_direction_toward_wrist = (Byte)0x01,
            myohw_x_direction_toward_elbow = (Byte)0x02,
            myohw_x_direction_unknown = (Byte)0xff
        }
        

        /// Possible outcomes when the user attempts a sync gesture.
        public enum myohw_sync_result_t
        {
            myohw_sync_failed_too_hard = (Byte)0x01,            ////< Sync gesture was performed too hard.
        }


        /// Classifier event data received in a myohw_att_handle_classifier_event attribute.
        public struct myohw_classifier_event_t
        {
            Byte type;                                          ////< See myohw_classifier_event_type_t

            /// Event-specific data
            [StructLayout(LayoutKind.Explicit)]
            struct MYOHW_PACKEDU
            {
                /// For myohw_classifier_event_arm_synced events.

                [FieldOffset(0)] MYOHW_PACKED MYOHW_PACKED;

                /// For myohw_classifier_event_pose events.
                [FieldOffset(0)] UInt16 pose;                   ////< See myohw_pose_t

                /// For myohw_classifier_event_sync_failed events.
                [FieldOffset(0)] Byte sync_result;              ////< See myohw_sync_result_t.
            }
        }

        public struct MYOHW_PACKED
        {
            public Byte arm;                                    ////< See myohw_arm_t
            public Byte x_direction;                            ////< See myohw_x_direction_t
        }


        /// Raw EMG data received in a myohw_att_handle_emg_data_# attribute.
        /// Value layout for myohw_att_handle_emg_data_#.
        public struct myohw_emg_data_t
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public SByte[] sample1;//[8];       ///< 1st sample of EMG data.
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public SByte[] sample2;//[8];       ///< 2nd sample of EMG data.
        }
        
        #endregion

    }
}
