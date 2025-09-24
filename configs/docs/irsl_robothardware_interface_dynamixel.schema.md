
#  Schema

```
```

Robot interface configuration for controlling Dynamixel motors using shared memory and hardware interface settings.

| Abstract | Extensible | Status | Identifiable | Custom Properties | Additional Properties | Defined In |
|----------|------------|--------|--------------|-------------------|-----------------------|------------|
| Can be instantiated | No | Experimental | No | Forbidden | Forbidden | [irsl_robothardware_interface_dynamixel.schema.json](irsl_robothardware_interface_dynamixel.schema.json) |

#  Properties

| Property | Type | Required | Defined by |
|----------|------|----------|------------|
| [HardwareIFSettings](#hardwareifsettings) | `object` | **Required** |  (this schema) |
| [SHMSettings](#shmsettings) | `object` | **Required** |  (this schema) |

## HardwareIFSettings

Settings for the hardware interface used to communicate with Dynamixel motors.

`HardwareIFSettings`
* is **required**
* type: `object`
* defined in this schema

### HardwareIFSettings Type


`object` with following properties:


| Property | Type | Required |
|----------|------|----------|
| `baud_rate`| integer | **Required** |
| `joint`| array | **Required** |
| `period`| number | **Required** |
| `port_name`| string | **Required** |



#### baud_rate

Baud rate for serial communication (e.g., 57600, 1000000).

`baud_rate`
* is **required**
* type: `integer`

##### baud_rate Type


`integer`








#### joint

List of joints (motors) connected via this hardware interface.

`joint`
* is **required**
* type: `object[]`


##### joint Type


Array type: `object[]`

All items must be of the type:
`object` with following properties:


| Property | Type | Required |
|----------|------|----------|
| `CommunicationGroupName`| string | Optional |
| `DynamixelSettings`| object | Optional |
| `ID`| integer | **Required** |



#### CommunicationGroupName

Name used to group motors for synchronized communication.

`CommunicationGroupName`
* is optional
* type: `string`

##### CommunicationGroupName Type


`string`








#### DynamixelSettings

Settings specific to the Dynamixel motor.

`DynamixelSettings`
* is optional
* type: `object`

##### DynamixelSettings Type

Unknown type `object`.

```json
{
  "type": "object",
  "description": "Settings specific to the Dynamixel motor.",
  "properties": {
    "Operating_Mode": {
      "type": "integer",
      "description": "Defines the motor's control mode. For example, in the XL430 model: 1 = Velocity Control, 3 = Position Control, 4 = Extended Position Control, 16 = PWM Control."
    },
    "Torque_Enable": {
      "type": "integer",
      "description": "Enable (1) or disable (0) the motor torque."
    },
    "Goal_Position": {
      "type": "integer",
      "description": "Target position value."
    },
    "Goal_Velocity": {
      "type": "integer",
      "description": "Target velocity value."
    },
    "Goal_Current": {
      "type": "integer",
      "description": "Target current value."
    },
    "Profile_Acceleration": {
      "type": "integer",
      "description": "Acceleration for profile-based motion planning."
    },
    "Profile_Velocity": {
      "type": "integer",
      "description": "Velocity for profile-based motion planning."
    },
    "Position_P_Gain": {
      "type": "integer",
      "description": "Proportional gain for position control."
    },
    "Position_I_Gain": {
      "type": "integer",
      "description": "Integral gain for position control."
    },
    "Position_D_Gain": {
      "type": "integer",
      "description": "Derivative gain for position control."
    },
    "Feedforward_1st_Gain": {
      "type": "integer",
      "description": "1st-order feedforward gain (velocity)."
    },
    "Feedforward_2nd_Gain": {
      "type": "integer",
      "description": "2nd-order feedforward gain (acceleration)."
    },
    "Current_Limit": {
      "type": "integer",
      "description": "Maximum allowable motor current."
    },
    "PWM_Limit": {
      "type": "integer",
      "description": "Maximum allowable PWM output."
    },
    "Velocity_Limit": {
      "type": "integer",
      "description": "Maximum allowable velocity."
    },
    "Max_Position_Limit": {
      "type": "integer",
      "description": "Upper limit of the position range."
    },
    "Min_Position_Limit": {
      "type": "integer",
      "description": "Lower limit of the position range."
    },
    "Temperature_Limit": {
      "type": "integer",
      "description": "Maximum allowed temperature before error shutdown."
    },
    "additionalProperties": true
  },
  "simpletype": "`object`"
}
```







#### ID

Unique ID of the Dynamixel motor.

`ID`
* is **required**
* type: `integer`

##### ID Type


`integer`
















#### period

Control loop period in seconds (e.g., 0.001 for 1 kHz).

`period`
* is **required**
* type: `number`

##### period Type


`number`








#### port_name

Name of the serial port used to communicate with the motors (e.g., '/dev/ttyUSB0').

`port_name`
* is **required**
* type: `string`

##### port_name Type


`string`











## SHMSettings

Settings related to shared memory (SHM) for inter-process communication.

`SHMSettings`
* is **required**
* type: `object`
* defined in this schema

### SHMSettings Type


`object` with following properties:


| Property | Type | Required |
|----------|------|----------|
| `hash`| integer | **Required** |
| `jointType`| array | **Required** |
| `shm_key`| integer | **Required** |



#### hash

Hash value used to validate the consistency of shared memory structure.

`hash`
* is **required**
* type: `integer`

##### hash Type


`integer`








#### jointType

List of data types exchanged via shared memory for each joint.

`jointType`
* is **required**
* type: `enum[]`


##### jointType Type


Array type: `enum[]`

All items must be of the type:
`string`



  
Type of joint data shared. Must be one of the predefined data types.









#### shm_key

Key used to identify the shared memory segment.

`shm_key`
* is **required**
* type: `integer`

##### shm_key Type


`integer`










