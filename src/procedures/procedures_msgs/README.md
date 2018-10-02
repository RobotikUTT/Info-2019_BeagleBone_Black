

# Package procedures_msgs

## But du package
Ce package permet de définir tout les [messages](#Msgs) et [Action](#Actions) liée au namespace AI. 

## Structure du package

### Msgs : 
Les différents messages défini dans ce package sont : 

 - [APliers](#APliers): message définissant une action Pince
 - [OrPoint](#OrPoint): message passant un point 2D orienté
 - [MPoint](#MPoint): message définissant un move Point

#### APliers

| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| Uint8 | action | X | 0: release block <br> 1: take block <br> 2: set pliers |
| Uint8 | level | X | niveau 0 à 4 |

#### OrPoint

| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int16 | x | mm | |
| int16 | y | mm | |
| int16 | rot | mRad | |

#### MPoint
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| OrPoint | Opoint | X |  |
| int8 | type | X | 0: Go To Angle <br> 1: Go To <br> 2: Rotation <br> 3: Rotation No Modulo |
| int8 | direction | X | -1: Backward <br> 0: Default <br> 1: Forward |
| int16 | timeout | sec | 0: no timeout |


### Actions : 

**Tout les actions doivent avoir le même message de retour.**

Les différents actions défini dans ce package sont : 

 - [Ball](#Ball): message définissant une action Ball
 - [Block](#Block): message définissant une action Block
 - [Canon](#Canon): message définissant une action Canon
 - [Move](#Move): message définissant une action Move
 - [Pliers](#Pliers): message définissant une action Pliers

#### Ball
##### Goal
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| [OrPoint](#OrPoint)| tube_pose | X |  |
| [OrPoint](#OrPoint)| shoot_pose | X |  |
| int16[]| param | X | [approach_angle, y_delta] |
##### Feedback
NONE

##### Result
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| bool| done | X |  |
| int8 | points_done | X | |

#### Block
##### Goal
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| [OrPoint](#OrPoint)| block_action | X |  |
| [OrPoint](#OrPoint)| depot | X |  |
##### Feedback
NONE

##### Result
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| bool| done | X |  |
| int8 | points_done | X | |

#### Canon
To Do
##### Goal
Empty
##### Feedback


##### Result
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| bool| done | X |  |
| int8 | points_done | X | |


#### Move

##### Goal
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| [MPoint[]](#MPoint)| points | X |  |
##### Feedback


##### Result
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| bool| done | X |  |
| int8 | points_done | X | |

#### Pliers

##### Goal
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| [APliers[]](#APliers)| act | X |  |
##### Feedback


##### Result
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| bool| done | X |  |
| int8 | points_done | X | |
