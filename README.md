

# Beagle Bone Black : le code haut niveau

Pour générer la documentation, taper dans un terminal à la racine du projet :

    doxygen Doxygen/Doxyfile
    
Un lien symbolique,  vers la page généré s’appelant *doc.html* vous ouvrira la documentation sur un navigateur.



## Introduction :

Le code haut niveau permet au robot d'ordonnancer les ses actions en ayant une certaine intelligence.
Le code s'exécute sur un Ubuntu 16 sur une Beagle Bone Black la (BBB).

La BBB communique avec le reste du robot par un bus CAN.

XXXX



## Structure du Repo

src : le code source

launch : les fichiers pour lancer le projet

param : les fichiers de parametre du projet

build & devel & install : dossier que ROS créer lors de la compilation du projet.

## Structure de projet 

Le projet source est découpé en "NameSpace" et en "Package".

Les Packages sont des dossiers contenant des codes sources, des librairie, ou différent ficher permettant d’exécuter le package.  Ils doivent respecter une certaine architecture pour que ROS puissent les compiler. 

Les NameSpaces permettent de regroupé différent Packages dans un même espace de travail. Elle sont s


Source:
[Packages](http://wiki.ros.org/Packages)
[ROS Names](http://wiki.ros.org/Names)
