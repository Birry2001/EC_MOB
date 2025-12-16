

## 5) Procédure pas à pas

### 5.1 Initialiser le dépôt Git local

Depuis la racine `Projet_de_synthese/` :

```bash
cd /chemin/vers/Projet_de_synthese
git init
````

---

### 5.2 Créer le `.gitignore` (AVANT le premier commit)

⚠️ Point critique : sans `.gitignore`, Git peut embarquer **build/install/log** (milliers de fichiers inutiles)

Créer/éditer le fichier .gitignore

Contenu recommandé( voir dans le dépot) :


---

### 5.3 Vérifier ce qui sera versionné

~~~bash
git add .
git status
~~~

---

### 5.4 Premier commit

~~~bash
git commit -m "Initial commit: structure du projet"
~~~

Notion clé :

git add = mettre dans la “zone de staging”
git commit = créer une sauvegarde locale (version)
git push = envoyer vers le dépôt distant

---


## 6) Problèmes rencontrés + solutions

### Problème A — “embedded git repository” (dépôt Git imbriqué)

**Symptôme :**
Lors de `git add .`, Git affiche un avertissement indiquant qu’un sous-dossier contient déjà un `.git` (ex: un driver ou un package copié depuis un autre repo).

**Pourquoi c’est grave :**
Si tu commits dans cet état, Git peut enregistrer ce sous-dossier comme un *sous-module implicite / coquille*, et ton binôme risque de cloner un dossier **vide** ou incomplet.

**Solution (monorepo) :**

1. Annuler le staging actuel :

~~~bash
git reset
~~~

2. Supprimer les `.git` internes (sans toucher celui de la racine) :

~~~bash
find project_ws DV -name ".git" -type d -exec rm -rf {} +
~~~

3. Refaire l’ajout + vérif :

~~~bash
git add .
git status
~~~

4. Commit :

~~~bash
git commit -m "Initial commit: ajout du code source complet (drivers + motion comp)"
~~~

---

### Problème B — artefacts colcon trackés par erreur (build/install/log)

**Symptôme :**
`git status` montre des milliers de fichiers, souvent dans `build/`, `install/`, `log/`.

**Cause :**
`.gitignore` absent/incomplet ou créé après compilation.

**Solution :**

1. Corriger `.gitignore`
2. Si des fichiers sont déjà trackés, les retirer de l’index :

~~~bash
git rm -r --cached project_ws/build project_ws/install project_ws/log
git commit -m "chore: stop tracking colcon artifacts"
~~~

---

### Problème C — “invalid reference” lors du checkout de `dev`

**Symptôme :**
`git checkout dev` échoue car Git local ne “voit” pas la branche distante.

**Cause :**
Les refs distantes ne sont pas encore récupérées.

**Solution :**

~~~bash
git fetch
git checkout dev
~~~

Attendu :

> Branch 'dev' set up to track remote branch 'dev' from 'origin'

---

## 7) Dépôt distant + premier push

### 7.1 Créer le repo distant (GitHub/GitLab)

Bonnes pratiques :

* créer un repo **vide**
* ne pas cocher “Initialize with README” (si déjà créé en local)
* récupérer l’URL (HTTPS ou SSH)

### 7.2 Renommer la branche par défaut en `main` (standard)

~~~bash
git branch -M main
~~~

### 7.3 Lier le remote + pousser

~~~bash
git remote add origin <URL_DU_DEPOT>
git push -u origin main
~~~

---

## 8) Mise en place d’un workflow `dev`

Deux cas :

### Cas 1 — Créer `dev` en local puis pousser

~~~bash
git switch -c dev
git push -u origin dev
~~~

### Cas 2 — `dev` existe déjà sur le remote

~~~bash
git fetch
git checkout dev
~~~

---

## 9) Notions Git mobilisées (rappel)

* **Repository local** : historique sur ta machine
* **Remote (`origin`)** : dépôt distant (GitHub/GitLab)
* **Staging area** : zone intermédiaire avant commit
* **Commit** : snapshot versionné
* **Branch** : ligne de développement (main stable / dev intégration)
* **Tracking branch** : lien local ↔ distant (ex: `dev` suit `origin/dev`)
* **.gitignore** : exclure des fichiers non pertinents
* **Monorepo** : un seul dépôt pour code + docs + ressources

---

## 10) Checklist de validation (fin d’étape)

* [ ] `git status` => “working tree clean”
* [ ] `.gitignore` présent et ignore `build/install/log`
* [ ] aucun `.git` imbriqué dans les sous-dossiers (sauf à la racine)
* [ ] `git log --oneline` montre au moins 1 commit initial
* [ ] `git remote -v` affiche `origin`
* [ ] `git push -u origin main` effectué avec succès
* [ ] branche `dev` prête (créée + trackée) si workflow prévu

---

## 11) Commandes utiles (mémo)

~~~bash
# Vérifier branches
git branch -a

# Vérifier remote
git remote -v

# Voir ce qui est ignoré (utile pour debug .gitignore)
git status --ignored

# Annuler staging
git reset

# Retirer des fichiers déjà trackés mais désormais ignorés
git rm -r --cached <path>

# Récupérer infos remote (branches distantes)
git fetch
~~~

---

## 12) Résultat attendu

À la fin de l’étape, le projet dispose :

* d’un **dépôt Git sain** à la racine,
* d’un `.gitignore` ROS 2 correct,
* d’un **historique initial** (commit 0),
* d’un dépôt distant synchronisé (`main`),
* d’un workflow prêt pour travailler en équipe (`dev`).

---
