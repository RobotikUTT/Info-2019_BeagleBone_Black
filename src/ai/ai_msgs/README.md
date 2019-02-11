# Package ai_msgs

## But du package
Ce package permet de définir tout les [messages](#Msgs) et [services](#Srvs) liée au namespace AI. 

## Structure du package

### Msgs : 
Les différents messages défini dans ce package sont : 

 - [ProximityStop](#ProximityStop): message contrôlant l'arret d'urgence
 - [NodeStatus](#NodeStatus): message publiant le status de chaque node
 - [Point2D](#Point2D): message passant un point 2D
 - [RobotStatus](#RobotStatus): message publiant le statut du robot
 - [SetSide](#SetSide): message publiant le camp du robot

#### *ProximityStop*
used between :

 - node ==> node

| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| Bool | proximity_set | X | 0: proximity off <br> 1: proximity on |

#### *NodeStatus*
used between :

 - node ==> node

| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| String[] | nodes_init | X | name of node in init |
| String[] | nodes_ready | X | name of node ready to work|
| String[] | nodes_error | X | name of node failed to init|
#### *Point2D*
used between :

 - node ==> node

| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int16 | x | mm |  |
| int16 | y | mm |  |
#### *RobotStatus*
used between :

 - node ==> node

| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| uint8 | robot_watcher | X | DEF STATE |
| uint8 | nodes_status | X | DEF STATE |
#### *SetSide*
used between :

 - node ==> node

| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| bool | side | X | 0: Green side <br> 1: Orange side |

### Srvs : 

Les différents service défini dans ce package sont : 

 - [CurrentActionDone](#CurrentActionDone): Demande de valider l'action en cours
 - [GetActionToDo](#GetActionToDo): Demande une action à exécuter
 - [NodeReadiness](#NodeReadiness): Demande de set le statut d'un node
 - [ObjectManager](#ObjectManager): To Do
 - [Pathfinder](#Pathfinder): WIP

#### *CurrentActionDone*
Used in : 

 - List item

##### request
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| Bool | done | X | S |

##### response
NONE
#### *GetActionToDo*
##### request
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int16 | robot_pos_x | mm |  |
| int16 | robot_pos_y | mm |  |

##### response
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| int8 | action_val | X | DEF STATE |
| [procedures_msgs/MPoint]() | point | X | TO DO |
| [procedures_msgs/OrPoint]() | action_pos | X | TO DO |
| [procedures_msgs/OrPoint]()| depot_pos | X | TO DO |
| int16[] | param | X | Array to pass different parameters |
#### *NodeReadiness*
##### request
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| string | node_name | X |  |
| bool | ready | X |  |
| int8 | error_code | X | To Do |
##### response
NONE
#### *ObjectManager*
##### request
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| string | object_name |  |  |
| [Point2D](#Point2D) | points |  |  |
| int8 | action |  |  |

##### response
NONE
#### *Pathfinder*
##### request
| Type | Var name | Units | comments |
|--|:--:|:--:|--|
| [Point2D](#Point2D) | from | X |  |
| [Point2D](#Point2D) | to | X |  |
| [Point2D](#Point2D) | robot_pose | X |  |

##### response
ToDo


