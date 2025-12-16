

# LIMO-DAVIS : Event-Based Dynamic Obstacle Detection for Robust Navigation

##  Présentation générale

Ce dépôt contient le projet de **Master 2 en robotique / perception artificielle**, dont l’objectif est d’**améliorer la robustesse de la navigation autonome du robot AgileX LIMO** en conditions difficiles, en intégrant une **caméra à évènements DAVIS-346** au sein de la pile ROS 2 existante.

Le projet s’inscrit dans une démarche **expérimentale et comparative**, en évaluant les performances des piles SLAM + Navigation actuelles du LIMO, puis en proposant une **brique évènementielle de détection d’obstacles dynamiques** intégrée à **Nav2**.

---

##  Objectifs du projet

### Objectif principal

> Améliorer les performances globales **SLAM + Navigation** du robot LIMO en conditions dégradées (faible luminosité, HDR, mouvements rapides, obstacles dynamiques), grâce à l’exploitation d’une caméra à évènements.

### Objectifs spécifiques

* Évaluer quantitativement les piles existantes :

  * **Cartographer LiDAR + Nav2**
  * **RTAB-Map RGB-D + Nav2**
* Implémenter une méthode de **détection d’obstacles dynamiques basée sur évènements**
* Intégrer cette détection dans la **costmap Nav2**
* Comparer navigation **avec / sans DAVIS** sur des scénarios identiques

---

##  Plateforme robotique

* **Robot** : AgileX **LIMO ROS 2**
* **OS** : Ubuntu 22.04
* **Middleware** : ROS 2 Humble
* **Calcul embarqué** : Intel NUC i7

### Capteurs utilisés

* **LiDAR 2D** : EAI T-mini Pro
* **Caméra RGB-D** : Orbbec Dabai
* **IMU** : IMU embarquée LIMO
* **Caméra à évènements** : **DAVIS-346** (events + APS + IMU)

---

##  Approche scientifique

### Méthode évènementielle retenue

La brique principale de perception dynamique est basée sur l’article :

> **Zhao, Li, Lyu – “Event-based Real-time Moving Object Detection Based on IMU Ego-motion Compensation”**,
> *IEEE ICRA 2023*

Principes clés :

1. **Compensation du mouvement propre (ego-motion)** à partir de l’IMU
   → warping non linéaire des évènements
2. **Segmentation dynamique** via *time image* et *count image*
3. **Clustering des objets en mouvement** (DBSCAN + information de flot)
4. **Projection des objets dynamiques** dans une couche de costmap Nav2

---


##  Organisation actuelle du dépôt 



---


## 📚 Références principales

* Zhao et al., *Event-based Real-time Moving Object Detection Based on IMU Ego-motion Compensation*, ICRA 2023
* RTAB-Map ROS
* Cartographer ROS
* Nav2 Documentation
* AgileX LIMO ROS 2 User Manual

---

## 👤 Auteurs

**Nochi Magouo**
Master 2 Robotique – Perception Artificielle
Université Clermont Auvergne / Institut Pascal


**Nadjib MEKELLECHE**
Master 2 Robotique – Perception Artificielle
Université Clermont Auvergne / Institut Pascal