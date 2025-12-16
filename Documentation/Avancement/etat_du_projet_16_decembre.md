Je pars de **tout ce qu’on s’est dit jusqu’ici** + quelques docs officiels (AgileX, RTAB-Map, Cartographer, Zhao 2023…). Voici un **rapport complet** de ton projet à l’instant T.

---

# 1. Contexte général du projet

* **Plateforme** : robot mobile **AgileX LIMO ROS2**

  * Version ROS2 avec **Intel NUC i7** sous **Ubuntu 22.04 + ROS 2 Humble**.([Agilex Robotics][1])
  * Pensée comme plateforme d’enseignement/recherche en navigation autonome.

* **Capteurs principaux sur ta version**

  * **LiDAR** : EAI **T-mini Pro** 360° (télémètre 2D) pour SLAM 2D & évitement d’obstacles.([Génération Robots][2])
  * **Caméra RGB-D** : **Orbbec Dabai** pour SLAM visuel 3D (RTAB-Map).([generationrobots.com][3])
  * **IMU** : centrale inertielle (type HI226) pour attitude et odométrie inertielle.([Ghostysky][4])
  * **Caméra à évènements DAVIS-346** (events + image APS + IMU intégrée) ajoutée dans **le cadre du projet**, pas dans la config de base du LIMO.

* **Mécanique et mobilité**

  * 4 modes de locomotion : **différentiel 4 roues, Ackermann, chenilles, Mecanum**.([Génération Robots][2])
  * Dimensions ≈ 322 × 220 × 251 mm ; masse ≈ 4,8 kg.([wevolver.com][5])

* **Contexte pédagogique / scientifique**

  * Projet de **Master 2 orienté recherche** (perception & robotique).
  * Objectif à moyen terme : profil R&D / thèse en robotique (SLAM, navigation, perception).

---

# 2. Objectif scientifique du projet

## 2.1 Objectif global

> **Améliorer la robustesse et les performances de la navigation autonome du LIMO en conditions difficiles**
> en exploitant une caméra à évènements DAVIS-346 pour **détecter les obstacles dynamiques** et enrichir la pile **SLAM + Nav2** déjà présente sur le robot.

Conditions difficiles ciblées :

* mouvements rapides du robot,
* scènes **HDR** (fortes variations lumineuses),
* **faible éclairage**,
* environnements **peu texturés**,
* présence d’**obstacles dynamiques** (personnes, objets en mouvement).

## 2.2 Axes de travail

Le projet est structuré en **deux grands axes** :

1. **Axe 1 – Évaluation rigoureuse des piles SLAM + Nav2 existantes sur le LIMO**

   * en particulier :

     * **Cartographer LiDAR + Nav2** (SLAM 2D)([Cartographer ROS][6])
     * **RTAB-Map RGB-D + Nav2** (SLAM visuel 3D)([introlab.github.io][7])
   * avec des **métriques quantitatives** (ATE/RPE, taux de réussite de navigation, replanifications, collisions, temps, etc.).

2. **Axe 2 – Détection d’obstacles dynamiques basée DAVIS-346 + IMU**

   * à partir de l’article **Zhao, Li, Lyu, ICRA 2023** *Event-based Real-time Moving Object Detection Based On IMU Ego-motion Compensation*.([Northwestern Polytechnical University][8])
   * intégration de cette brique dans **Nav2** via une **couche dynamique de costmap**.

La **comparaison principale** se fera entre :

* **Pipeline A (référence)** : RTAB-Map RGB-D + Nav2, sans DAVIS.
* **Pipeline B (nouveau)** : RTAB-Map + Nav2 + brique **Zhao 2023** basée DAVIS-346.

---

# 3. Piles logicielles existantes sur le LIMO

## 3.1 Environnement logiciel

* OS : **Ubuntu 22.04**.
* Middleware : **ROS 2 Humble**.
* LIMO ROS2 fournit une stack pré-intégrée pour SLAM et navigation (LiDAR + RGB-D + IMU) adaptée à la recherche.([Agilex Robotics][1])

Tu as notamment :

* Paquets bas niveau (type `limo_base`) pour le driver, `/cmd_vel`, `/odom`, `/imu`, `/tf`, etc.
* Paquets de **bringup** (`limo_bringup`) pour lancer les démos :

  * SLAM LiDAR Cartographer,
  * SLAM visuel RTAB-Map,
  * Navigation avec Nav2,
  * Démos de suivi de ligne, QR code, etc. (vision classique).

## 3.2 SLAM LiDAR + Nav2 (Cartographer)

* **Cartographer** (Google) : système de SLAM 2D/3D en temps réel, avec intégration ROS (cartographer_ros).([Cartographer ROS][6])

* Sur le LIMO, il est utilisé pour :

  * cartographie 2D à partir du **LiDAR T-mini Pro**,
  * localisation + fourniture de `/map` → Nav2.

* Nav2 utilise alors :

  * une **global costmap** et une **local costmap**,
  * un planificateur global (ex. A*, Dijkstra) + un contrôleur local (genre DWB/TEB),
  * pour atteindre des objectifs en évitant les obstacles.([docs.nav2.org][9])

## 3.3 SLAM visuel RGB-D + Nav2 (RTAB-Map)

* **RTAB-Map** : approche SLAM **graph-based** pour RGB-D / stéréo / LiDAR, avec détection de boucles par apparence.([introlab.github.io][7])
* **rtabmap_ros** fournit l’intégration ROS 1/ROS 2 ; la branche ROS2 supporte Humble.([GitHub][10])

Sur le LIMO :

* la caméra **Orbbec Dabai** alimente RTAB-Map via un flux RGB-D,
* RTAB-Map produit :

  * une carte 3D (nuages de points),
  * une carte 2D projetée,
  * une pose du robot dans `/map`.
* Ces sorties alimentent Nav2 pour la navigation **visuelle**.

**Conclusion** : tu disposes déjà de deux pipelines SLAM + Nav2 fonctionnels, dont le **pipeline visuel RTAB-Map + Nav2** sera la **référence** principale pour évaluer ton ajout DAVIS.

---

# 4. Problématique scientifique détaillée

Les limites classiques de la navigation basées sur caméras « frame-based » et LiDAR en conditions difficiles :

* **Motion blur** en cas de mouvements rapides → perte de features pour SLAM visuel.
* **Faible lumière / HDR** → images bruitées ou saturées.
* **Environnements peu texturés** (murs blancs, couloirs uniformes) → peu de points d’intérêt.
* **Obstacles dynamiques** : SLAM et costmap standard ne distinguent pas forcément bien ce qui est **statique** de ce qui est **mobile**, ce qui peut induire des plans sous-optimaux ou des collisions.

Les **caméras à évènements** (type DAVIS-346) répondent bien à ces limitations :

* latence très faible,
* fonctionnement à très grande dynamique,
* focalisation sur les **changements** (mouvements), donc idéales pour détecter les objets en mouvement même quand l’image classique est floue ou sombre.

Ton projet vise donc à **exploiter ces avantages pour la navigation** :
non pas en refaisant tout le SLAM, mais en ajoutant une **brique de détection d’obstacles dynamiques** couplée à la costmap Nav2.

---

# 5. Méthode évènementielle choisie : Zhao et al., ICRA 2023

## 5.1 Référence

* C. Zhao, Y. Li, Y. Lyu,
  **“Event-based Real-time Moving Object Detection Based On IMU Ego-motion Compensation”**,
  Proc. IEEE ICRA 2023, pp. 690–696.([Northwestern Polytechnical University][8])

## 5.2 Idée générale de l’article

But :

> Détecter **en temps réel** les objets en mouvement vus par une caméra à évènements **sur une plateforme mobile** (qui bouge elle aussi), en exploitant l’IMU pour compenser le **mouvement propre** (ego-motion).

Contributions principales :

1. **Compensation d’ego-motion par warping non linéaire**

   * Utilisation des mesures gyroscopiques de l’IMU pour modéliser la rotation du capteur entre (t_0) et (t).
   * Application d’une fonction de **warping non linéaire** (géométrie projective) pour mapper chaque évènement ((x, y, t)) vers une position compensée à un temps de référence (typiquement la fin de la fenêtre).
   * Résultat : les évènements du **fond statique** s’alignent, tandis que ceux des objets mobiles restent « flous »/déplacés.([Semantic Scholar][11])

2. **Segmentation dynamique basée sur count image + time image**

   * Construction d’une **count image** (C(x, y)) (nombre d’événements par pixel) et d’une **time image** (T(x, y)) (timestamp moyen).
   * Après compensation :

     * le fond statique a des timestamps moyens homogènes,
     * les objets en mouvement ont des timestamps plus récents ou plus dispersés.
   * Un **seuil adaptatif** (fonction de la norme de la vitesse angulaire IMU) sépare les pixels dynamiques des statiques.([Semantic Scholar][11])

3. **Clustering des objets dynamiques**

   * Calcul d’un **pseudo-flot optique** pour les pixels dynamiques.
   * Utilisation d’un **DBSCAN modifié** qui tient compte à la fois :

     * de la proximité spatiale des évènements,
     * de la similarité de vitesse.
   * Chaque cluster correspond à un **objet dynamique**.

4. **Performance temps réel**

   * L’article montre que la méthode fonctionne **en temps réel** sur des plateformes type NUC avec des caméras DAVIS/DVXplorer, et qu’elle surpasse d’autres méthodes IMU-aided en qualité de compensation et densité d’évènements utiles.([Semantic Scholar][11])

## 5.3 Adaptation prévue pour ton projet

Dans ton projet, cette méthode devient :

1. **DAVIS-346 + IMU (LIMO ou DAVIS)**

   * flux `events` (x, y, t, polarité),
   * flux `imu` (ωx, ωy, ωz).

2. **Node ROS2 `motion_compensation`**

   * implémentation de la **fonction de warping Zhao 2023**
   * sortie : événements compensés + count image + time image.

3. **Node ROS2 `dynamic_segmentation`**

   * segmentation binaire statique/dynamique via threshold adaptatif sur `T(x,y)`, éventuellement couplé à `C(x,y)` (filtrage de bruit).

4. **Node ROS2 `dynamic_clustering`**

   * clustering (connected components ou DBSCAN) des zones dynamiques → listes d’objets.

5. **Node ROS2 `davis_costmap_layer` (plugin Nav2)**

   * projette les clusters dynamiques en **zones de coût élevé** dans la costmap 2D, pendant un temps court.

Ce pipeline remplace 0-MMS comme méthode principale (0-MMS reste seulement dans l’état de l’art).

---

# 6. Architecture ROS 2 cible du système complet

## 6.1 Vue globale des capteurs & piles

**Capteurs :**

* LiDAR T-mini Pro → `/scan`
* Caméra RGB-D Orbbec Dabai → `/rgb/image`, `/depth/image`, `/camera_info`
* IMU LIMO → `/imu`
* DAVIS-346 → `/dvs/events`, `/dvs/imu` (+ éventuellement APS)

**Pipes existants :**

1. **SLAM LiDAR (Cartographer)**
   `/scan`, `/imu` → Cartographer → `/map`, `/odom`, `/tf` → Nav2.

2. **SLAM visuel (RTAB-Map)**
   `/rgbd` + odom → RTAB-Map → `/map`, `/odom`, `/tf` → Nav2.

**Nouveau pipe DAVIS :**

3. **Détection d’obstacles dynamiques (Zhao 2023)**

   * `/dvs/events` + `/dvs/imu` → `motion_compensation_node`
   * → `dynamic_segmentation_node`
   * → `dynamic_clustering_node`
   * → `/davis_dynamic_obstacles` → plugin costmap Nav2 `DavisDynamicLayer`.

## 6.2 Deux pipelines de navigation à comparer

### Pipeline A – Baseline

* RTAB-Map RGB-D fournit `/map` + pose.
* Nav2 avec costmaps standard (LiDAR + obstacles statiques).
* Pas de DAVIS.

### Pipeline B – Enrichi avec DAVIS

* RTAB-Map + mêmes réglages.
* Nav2 + **DavisDynamicLayer** qui :

  * lit `/davis_dynamic_obstacles`,
  * gonfle les coûts autour des obstacles dynamiques,
  * provoque des déviations de trajectoire / ralentissements.

Tu compareras **A vs B** sur les **mêmes parcours** et conditions.

---

# 7. Protocole d’évaluation prévu

## 7.1 Scénarios de test

Tu as identifié plusieurs scénarios :

1. **Parcours “normal” de référence**

   * bon éclairage, vitesse modérée, peu d’obstacles dynamiques.

2. **HDR / forte variation de lumière**

   * zones très éclairées / sombres, passages de lumière directe, etc.

3. **Faible éclairage**

   * couloir sombre, lumières faibles, où la caméra RGB-D se dégrade.

4. **Environnement peu texturé**

   * murs uniformes, sol homogène → peu de features pour SLAM visuel classique.

5. **Présence d’obstacles dynamiques**

   * personnes traversant la trajectoire, obstacles mobiles lents/rapides.

## 7.2 Métriques SLAM

Sur la base de ton experience M1 (LiDAR-SLAM), tu prévois :

* **ATE** (Absolute Trajectory Error) et **RPE** (Relative Pose Error) via `evo`.
* **Drift** par mètre.
* **Pertes de suivi / re-initialisations**.
* **Consommation CPU / RAM** si possible.

## 7.3 Métriques navigation (Nav2)

* **Taux de réussite** (arrive à la cible ou non).
* **Temps** pour atteindre l’objectif.
* **Longueur de trajectoire**.
* **Nombre de replanifications** (global planner).
* **Collisions / near-miss** (basé sur logs / observations).
* **Stabilité des vitesses linéaires et angulaires** (oscillations, arrêts brusques, etc.).

Ces métriques seront mesurées **pour A et B** sur les mêmes scénarios, afin de mettre en évidence l’apport (ou les limites) de la couche DAVIS.

---

# 8. Ressources logicielles de référence déjà identifiées

1. **Code de motion compensation (référence ROS1)**

   * Repo Git (zip) type `Jhonny-Li/Motion-compensation`, implémentant la motion compensation + traitement d’évènements pour Zhao 2023.
   * Pas de licence claire → **code utilisé comme référence pour ré-implémenter en ROS 2**, pas pour redistribution directe.

2. **RTAB-Map ROS2**

   * `rtabmap_ros` branch `ros2` avec support Humble.([GitHub][10])

3. **Cartographer ROS**

   * `cartographer_ros` (Apache 2.0) pour SLAM 2D LiDAR.([Cartographer ROS][6])

4. **Nav2 documentation**

   * docs sur SLAM + mapping & costmaps pour ROS2.([docs.nav2.org][9])

---

# 9. État d’avancement **réel** du projet

D’après nos échanges :

### 9.1 Clarifié / décidé

* ✅ **Objectif scientifique** clairement formulé : améliorer la navigation en conditions difficiles via DAVIS + obstacles dynamiques.
* ✅ **Deux axes principaux** définis :

  * évaluation SLAM + Nav2 existants,
  * développement brique DAVIS.
* ✅ **Pile de référence pour la comparaison** :

  * **RTAB-Map RGB-D + Nav2** (pipeline A).
* ✅ **Méthode évènementielle finale choisie** :

  * article **Zhao et al. 2023, ICRA** (et non 0-MMS).
* ✅ **Architecture logicielle cible** esquissée :

  * chaines de nœuds ROS2 pour motion compensation, segmentation, clustering, costmap layer.
* ✅ **Protocole de test** conceptualisé :

  * scénarios, métriques SLAM et Nav2.

### 9.2 En cours / à clarifier

* 🔄 **Intégration matérielle DAVIS-346 sur le LIMO**

  * montage mécanique, câblage, alimentation,
  * vérification de l’orientation par rapport au repère du robot.
* 🔄 **Choix et mise en place du driver DAVIS sous ROS 2**

  * driver natif ROS2 ou bridge ROS1→ROS2.
* 🔄 **Définition détaillée des messages ROS** entre :

  * `motion_compensation_node` → `dynamic_segmentation` → `dynamic_clustering` → `davis_costmap_layer`.

### 9.3 Pas encore commencé / à faire

* ⏳ **Calibration complète** :

  * calibration interne DAVIS (focale, distorsion),
  * calibration extrinsèque DAVIS ↔ IMU (si IMU LIMO utilisée),
  * synchronisation temporelle des capteurs (ROS2 + DAVIS).
* ⏳ **Portage / implémentation ROS 2 de Zhao 2023** :

  * re-codage du warping non linéaire en C++/Python ROS2,
  * construction count/time images en temps réel,
  * segmentation dynamique + clustering (CC ou DBSCAN).
* ⏳ **Développement du plugin costmap Nav2** :

  * implémentation d’une nouvelle couche `DavisDynamicLayer` (plugin C++ Nav2),
  * paramétrage de la fusion avec les autres couches (obstacles, inflation, etc.).
* ⏳ **Campagne d’expérimentations A vs B** :

  * enregistrement de rosbag2,
  * traitement des résultats (evo, scripts Python),
  * production de graphiques / tableaux.
* ⏳ **Rédaction structurée du mémoire / rapport** :

  * état de l’art (SLAM, event-based vision, Zhao 2023, etc.),
  * méthodologie, résultats, discussion, perspectives.

---

# 10. Prochaines étapes (priorisées)

Pour être concret, les **prochaines actions prioritaires** pourraient être :

1. **Matériel + driver DAVIS (bloquant pour tout le reste)**

   * fixer la DAVIS sur le LIMO,
   * faire fonctionner le driver (afficher les évènements en live dans RViz / rqt).

2. **Mini-démo de motion compensation seule**

   * adapter / réimplémenter la partie warping IMU + visualisation,
   * vérifier qualitativement qu’un fond statique devient net après compensation.

3. **Implémentation simple de segmentation dynamique**

   * count + time image, seuil adaptatif,
   * visualisation d’un masque binaire des zones dynamiques.

4. **Clustering simple + export en obstacles 2D**

   * connected components ou DBSCAN,
   * publication de cercles/boîtes en coordonnées robot.

5. **Plugin Nav2 minimal**

   * plugin qui, à partir des obstacles dynamiques, augmente le coût dans la costmap,
   * test sur un scénario simple avec un obstacle mobile.

6. **Début de l’évaluation A vs B**

   * un premier scénario simple (couloir, 1 obstacle mobile),
   * quelques runs enregistrés,
   * premiers graphiques comparatifs.

---

Si tu veux, je peux maintenant te générer à partir de ce rapport :

* soit une **version “académique” courte** (1 page) pour ton encadrante,
* soit un **squelette de chapitre de mémoire** (plan + sections LaTeX) basé sur cette synthèse.

[1]: https://global.agilex.ai/products/limo-ros2?utm_source=chatgpt.com "LIMO ROS2"
[2]: https://static.generation-robots.com/media/limo-ros-user-manual-en.pdf?utm_source=chatgpt.com "1 LIMO Introduction"
[3]: https://www.generationrobots.com/blog/en/agilex-robotics-lance-deux-nouvelles-versions-de-son-robot-limo-limo-pro-limo-ros2/?srsltid=AfmBOorli3px1nDSo0wioj5PcV34hTSwOMBcsmKYrtPWM_k3KvS8xVIC&utm_source=chatgpt.com "AgileX Robotics launches two new versions of its LIMO robot"
[4]: https://www.ghostysky.com/product/agilex-limo-ros2-intel-nuc/?utm_source=chatgpt.com "Agilex LIMO ROS2 (Intel NUC)"
[5]: https://www.wevolver.com/specs/agilex-limo?utm_source=chatgpt.com "AgileX Limo"
[6]: https://google-cartographer-ros.readthedocs.io/?utm_source=chatgpt.com "Cartographer ROS Integration — Cartographer ROS ..."
[7]: https://introlab.github.io/rtabmap/?utm_source=chatgpt.com "RTAB-Map | Real-Time Appearance-Based Mapping"
[8]: https://pure.nwpu.edu.cn/en/publications/event-based-real-time-moving-object-detection-based-on-imu-ego-mo/?utm_source=chatgpt.com "Event-based Real-time Moving Object Detection Based On IMU ..."
[9]: https://docs.nav2.org/setup_guides/sensors/mapping_localization.html?utm_source=chatgpt.com "Mapping and Localization"
[10]: https://github.com/introlab/rtabmap_ros?utm_source=chatgpt.com "introlab/rtabmap_ros: RTAB-Map's ROS package."
[11]: https://www.semanticscholar.org/paper/fa4923ea903111403f13c0f893846a12cd47a02a?utm_source=chatgpt.com "Event-based Real-time Moving Object Detection ..."
