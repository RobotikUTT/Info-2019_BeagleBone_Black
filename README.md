# Beagle Bone Black : le code haut niveau

Ce repo contient le code permettant au gros robot de fonctionner pour la coupe de france de robotique 2019. Une documentation plus avancée est disponible sur [le wiki](https://github.com/RobotikUTT/Info-2019_BeagleBone_Black/wiki).

## Télécharger et utiliser le projet

L'installation de ROS est nécessaire pour pouvoir compiler et éxécuter le projet, se référer [à la documentation officielle pour votre distribution](https://www.ros.org/install/). Une page du wiki est dédiée à la configuration d'une carte contenant le code.

Ceci fait, il faut simplement cloner le projet avec git dans le dossier voulu.
~~~~bash
git clone https://github.com/RobotikUTT/Info-2019_BeagleBone_Black.git
cd Info-2019_BeagleBone_Black
~~~~

Différents scripts dans le dossier `scripts` sont disponibles pour installer différentes dépendances. Etant donné les changements opérés sur ROS et les différences entre distributions, n'hésitez pas à adapter le script à votre distribution.
~~~~bash
./scripts/setup.sh
~~~~

A noter que le script installe catkin-tools, un utilitaire de compilation optionnel.

Si certains paquets sont manquants lors de la compilation (catkin build présente une erreur liée à un paquet ou une importation manquante) ou de l'éxécution (pour des erreurs python), il est possible que certaines dépendances soit manquantes du script d'installation, il convient alors de les installer.

Une autre possibilité pour l'éxecution est que le fichier `devel/setup.sh` ne soit pas sourcé.
~~~~bash
source ./devel/setup.sh # dans la racine du repo
~~~~

## Compilation du projet

Pour pouvoir compiler le projet, utilisez la commande fournies par catkin-tools.
~~~~
catkin build
~~~~

Il est aussi possible d'utiliser `catkin_make` si catkin-tools n'est pas installé (même résultat mais avec moins de style).

### Lancer les tests

Les tests unitaires peuvent être lancés à l'aide de la commande suivante :
~~~~bash
catkin run_tests
~~~~

Les résultats sont disponibles à l'aide de la commande `catkin_test_results`.

Il est également possible de lancer un fichier de test pour un noeud spécifique (des logs plus poussés mais moins lisibles sont disponibles en ajoutant l'option `-t`).
~~~~bash
rosrun [package] [test_file].launch
~~~~

## Lancer le robot

### En simulation

Création du can virtual pour émuler le bus de communication
~~~~
rosrun can_interface virtual_can.sh
~~~~

Lancement du robot configuré pour la simulation
~~~~
roslaunch launch/simulation.launch
~~~~

### En réel

~~~~
roslaunch launch/robot.launch
~~~~

## Documentation

Pour générer la documentation, taper dans un terminal à la racine du projet :
~~~~
doxygen Doxygen/Doxyfile
~~~~

Un lien symbolique, vers la page générée s’appelant *doc.html* vous ouvrira la documentation sur un navigateur.

### Différentes sources et documentations
- [Packages](http://wiki.ros.org/Packages)
- [ROS Names](http://wiki.ros.org/Names)
- [Tuto catkin](http://wiki.ros.org/catkin/Tutorials)
- [CMakeList.txt](http://wiki.ros.org/catkin/CMakeLists.txt)
