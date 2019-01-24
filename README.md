# Beagle Bone Black : le code haut niveau

Ce repo contient le code permettant au gros robot de fonctionner pour la coupe de france de robotique 2019.

## Compilation du projet

Pour pouvoir compiler le projet, tapez la commande suivant à la racine du repo.

~~~~
catkin_make
~~~~

Cette commande lancera la compilation de tout le projet. Faites attention que vos fichiers CmakeList et package.xml de chaque package soient bien configurés.


## Lancer le robot

En simulation:

Creer le can virtuel:
~~~~
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
~~~~

Vérifier dans le fichier *server_param.yaml* que la simulation soit activée, et que le nom de l'interface can correspond. Taper ensuite, à la racine du projet :
~~~~
roslaunch launch/robot.launch
~~~~
En reel:

Vérifier dans le fichier *server_param.yaml* que la simulation soit désactivée, et que le nom de l'interface can correspond. Taper ensuite, à la racine du projet :
~~~~
roslaunch launch/robot.launch
~~~~

## Documentation

Pour générer la documentation, taper dans un terminal à la racine du projet :
~~~~
doxygen Doxygen/Doxyfile
~~~~

Un lien symbolique, vers la page générée s’appelant *doc.html* vous ouvrira la documentation sur un navigateur.

### Introduction :

Le code haut niveau permet au robot d'ordonnancer les ses actions en ayant une certaine intelligence.
Le code s'exécute sur un Ubuntu 16 sur une Beagle Bone Black (BBB).

La BBB communique avec le reste du robot par un bus CAN.


### Structure du Repo

- src : le code source
- launch : les fichiers pour lancer le projet
- param : les fichiers de paramètre du projet
- build & devel & install : dossier que ROS créé lors de la compilation du projet.
- html : les fichiers générés par Doxygen
- Doxygen : les fichier configurant Doxygen

### Structure de projet 

Le projet source est découpé en "NameSpace" et en "Package".

Les Packages sont des dossiers contenant des codes sources, des librairie, ou différent ficher permettant d’exécuter le package. Ils doivent respecter une certaine architecture pour que ROS puissent les compiler.

Les NameSpaces permettent de regrouper différent Packages dans un même espace de travail. Le principe est similaire que les NameSpace en C++.

Dans le cadre du projet, nous avons explicité les namespaces en regroupant les différents packages dans des dossiers. Ces dossiers sont invisibles d'un point vue de la compilation, mais aide la lecture et la compréhension du projet.

Le projet ROS peut se découper de la sorte d'un point de vu logiciel.

~~~~
                           BUS CAN
                              ^
                              |
                      +-------v-------+
                      |               |
                      |   Socketcan   |
                      |               |
                      +---+------^----+
                          |      |
                          |      |
                      +---v------+----+
                      |               <-------------------+
                      |Robot_Interface|                   |
          +-----------+               +-----------+       |
          |           +---+------^----+           |       |
          |               |      |                |       |
          |               |      |                |       |
+---------v---+       +---v------+----+       +---v-------+-+
|             <-------+               +------->             |
| PathFinding |       |   Controller  |       |  Procedure  |
|             +------->               |       |             |
+-------------+       +--^----------^-+       +-------------+
                         |          |
                         |          |
           +-------------+-+     +--+------------+
           |               |     |               |
           |   Scheduler   |     | Robot_watcher |
           |               |     |               |
           +---------------+     +------+--^-----+
                                        |  |
                                        v  +
                                  USER INTERFACE
~~~~



Source:
[Packages](http://wiki.ros.org/Packages)
[ROS Names](http://wiki.ros.org/Names)
[Tuto](http://wiki.ros.org/catkin/Tutorials)
[Cmaklist](http://wiki.ros.org/catkin/CMakeLists.txt)
