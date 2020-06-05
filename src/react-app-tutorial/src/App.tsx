import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import Door from './Door';

function App() {
  const exampleDoor: RomiCore.Door = {
    name: 'example_door',
    door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING,
    motion_direction: 1,
    motion_range: 1,
    v1_x: 0,
    v1_y: 0,
    v2_x: 0,
    v2_y: 0,
  };
  const exampleDoorState: RomiCore.DoorState = {
    door_name: 'example_door',
    door_time: RomiCore.toRosTime(new Date()),
    current_mode: {
      value: RomiCore.DoorMode.MODE_CLOSED,
    },
  };

  return <Door door={exampleDoor} doorState={exampleDoorState} />;
}

export default App;
