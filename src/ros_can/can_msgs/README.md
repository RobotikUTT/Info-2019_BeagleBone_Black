# Package can_msgs


## But du package
Ce package permet de définir tout les [messages](#Msgs)  liées au namespace Can.

## Structure du package

### Msgs : 

Les différents messages défini dans ce package sont : 
- [ActionPliers](#ActionPliers): message donnant un ordre Pince.
- [CurSpeed](#CurSpeed): message contenant la vitesse du robot et des roues.
- [Finish](#Finish): message explicitant la fin d'une action.
- [Frame](#Frame): message du package soketcan pour envoyer un message CAN sur le bus.
- [ObjectOnMap](#ObjectOnMap): message donnant la position d'un objet sur la carte.
- [PID](#PID): message donnant les paramète d'un PID
- [Point](#Point): message donnant un point orienté dans la carte
- [PWMs](#PWMs): message donnant un ordre PWM.
- [RobotBlocked](#RobotBlockes): message disant que le robot est bloqué.
- [SonarDistance](#SonarDistance): message donnant les distances des sonars.
- [Speed](#Speed): message donnant un ordre de vitesse.
- [Status](#Status): message disant le status du robot.
- [STMParam](#STMParam): message partant les parametres de la STM.
- [ThrowBalls](#ThrowBalls): message donnant l'ordre de tirer les balles.
- [WheelsDistance](#WheelsDistance): message donnant les états des encodeurs.

#### ActionPliers
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int8 | action | X |  |
| int8 | level | X |  |

#### CurSpeed
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int16 | linear_speed | mm/s |  |
| int16 | left_speed | mm/s |  |
| int16 | right_speed | mm/s |  |

#### Finish
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int8 | val | X |  |

#### Frame
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| Header | header | X |  |
| uint32 | id | X |  |
| bool | is_rtr | X |  |
| bool | is_extended | X |  |
| bool | is_error | X |  |
| uint8 | dlc | X |  |
| uint8[8] | data | X |  |

#### ObjectOnMap
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int8 |object_id | X |  |
| int16 | x | mm |  |
| int16 | y | mm |  |
| int16 | radius | mm |  |

#### PID
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int16 | P | unit/1000 |  |
| int16 | I | unit/1000 |  |
| int16 | D | unit/1000 |  |

#### Point
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int16 | pos_x | mm |  |
| int16 | pos_y | mm |  |
| int16 | angle | mRad |  |
| int8 | direction | X |  |

#### PWMs
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int16 | left_pwm | mm |  |
| int16 | right_pwm | mm |  |

#### RobotBlocked
EMPTY

#### SonarDistance
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| uint8 | dist_front_left | mm |  |
| uint8 | dist_front_right | mm |  |
| uint8 | dist_back_left | mm |  |
| uint8 | dist_back_right | mm |  |

#### Speed
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int16 | linear_speed | mm/s |  |
| int16 | angular_speed | mm/s |  |
| uint16 | duration | ms |  |

#### Status
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| unit8 | status | X |  |

#### STMParam
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| uint16 | max_linear_speed | mm/s |  |
| uint16 | max_angular_speed | mm/s |  |
| uint16 | max_acc | mm/s² |  |


#### ThrowBall
EMPTY not used

#### WheelsDistance
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| uint16 | right_wheel_dist | mm |  |
| uint16 | left_wheel_dist | mm |  |
