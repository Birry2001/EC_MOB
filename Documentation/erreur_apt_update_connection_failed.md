````markdown
# Problème avec `apt-get update` — Erreurs “Connection failed [IP: …]”

## 🔎 Description du problème

Quand tu exécutes :

```bash
sudo apt-get update
````

tu peux obtenir des messages d’erreur comme :

```
Err: http://archive.ubuntu.com/ubuntu focal InRelease
  Connection failed [IP: 2a06:bc80:0:1000::18 80]
Err: http://security.ubuntu.com/ubuntu focal-security InRelease
  Connection failed [IP: 2a06:bc80:0:1000::17 80]
W: Failed to fetch … Connection failed [IP: … 80]
W: Some index files failed to download. They have been ignored, or old ones used instead.
```

Cela signifie qu’`apt-get` n’a pas réussi à se connecter aux dépôts — plus précisément, la tentative de connexion s’est faite via une adresse IPv6, et la connexion a échoué.

---

## 🌐 Cause(s) possible(s) & Concepts liés

* **IPv6 vs IPv4**
  L’adresse mentionnée (`2a06:bc80:…`) est une adresse IPv6. Si ton réseau, ton fournisseur d’accès ou ta configuration n’assure pas un support correct de l’IPv6, alors toute tentative de connexion via IPv6 échouera. ([Launchpad][1])
* **Préférence automatique pour IPv6**
  Par défaut, sur beaucoup de systèmes, quand un nom de domaine (comme `archive.ubuntu.com`) résout à la fois en IPv4 et IPv6, la pile réseau peut privilégier IPv6. Si l’IPv6 ne fonctionne pas réellement, cela conduit à des échecs de connexion. ([Ask Ubuntu][2])
* **Incompatibilité réseau ou routeur bloquant IPv6**
  Même si l’IPv6 est activée sur ta machine, le routeur ou fournisseur peut ne pas acheminer correctement le trafic IPv6 — ce qui rend la connexion impossible. ([Raspberry Pi Forums][3])

En résumé : le problème vient très vraisemblablement du fait qu’`apt-get` essaie d’utiliser IPv6 (via DNS + résolution), alors que la connectivité IPv6 n’est pas opérationnelle dans ton réseau.

---

## ✅ Solutions possibles

### Forcer `apt-get` à utiliser IPv4

* **Ponctuellement** (pour une seule commande) :

  ```bash
  sudo apt-get -o Acquire::ForceIPv4=true update
  sudo apt-get -o Acquire::ForceIPv4=true upgrade
  ```

  Cela force `apt-get` à utiliser IPv4, ce qui contourne les problèmes liés à IPv6. ([Ask Ubuntu][4])

* **De façon permanente** :

  Crée (ou modifie) un fichier de configuration pour qu’`apt` utilise systématiquement IPv4 :

  ```bash
  echo 'Acquire::ForceIPv4 "true";' | sudo tee /etc/apt/apt.conf.d/99force-ipv4
  ```

  Après cela, toutes les commandes `apt-get update/upgrade` utiliseront IPv4. ([Ask Ubuntu][5])

### (Optionnel) Préférer IPv4 au niveau système — sans désactiver complètement IPv6

Tu peux configurer la priorité d’adresses pour que le système préfère IPv4 quand IPv6 existe mais est problématique :

* Édite `/etc/gai.conf` et décommente (ou ajoute) :

  ```text
  precedence ::ffff:0:0/96  100
  ```

  Cela dit au système de privilégier les adresses IPv4 quand c’est possible — ce qui peut éviter ce genre de blocage tout en gardant l’IPv6 fonctionnelle si elle l’est. ([Ask Ubuntu][2])

### (Optionnel) Désactiver IPv6 globalement

Si tu n’as pas besoin d’IPv6 et que le réseau ne le supporte pas, tu peux le désactiver complètement au niveau système. Par exemple, via les paramètres réseau ou en modifiant la configuration sysctl pour désactiver IPv6. ([NameHero][6])

---

## 🧪 Diagnostics — Comment vérifier si le problème vient de l’IPv6

Voici quelques commandes utiles pour diagnostiquer :

```bash
# Vérifier si une adresse IPv6 est assignée à tes interfaces
ip -6 addr show

# Tester la connectivité IPv6 vers un domaine externe
ping6 google.com

# Tester apt-get en forçant IPv4 — si ça marche, c’est probablement un pb IPv6
sudo apt-get -o Acquire::ForceIPv4=true update

# Vérifier les résolutions DNS pour IPv4 et IPv6 d’un dépôt
getent ahosts archive.ubuntu.com
```

Si `ip -6 addr show` ne retourne rien ou seulement des adresses locales, ou si `ping6` échoue, cela indique clairement que ton IPv6 n’est pas fonctionnel. Si le test `apt-get` en IPv4 passe tandis que la version standard échoue, cela confirme le problème de connectivité IPv6.

---

## 🧑‍💻 Pourquoi ce problème est fréquent (contexte)

* Beaucoup de réseaux (particuliers ou professionnels) ne disposent **pas d’une IPv6 correctement configurée** — ou le FAI ne la supporte pas pleinement.
* Les systèmes modernes (et `apt`) *essaient d’utiliser IPv6 en priorité* dès qu’une adresse AAAA est disponible — ce qui entraîne des échecs automatiques si IPv6 est cassée.
* Les dépôts publics (mirrors Ubuntu, sécurité…) peuvent être accessibles en IPv4 mais **non accessibles en IPv6** — ce qui provoque des erreurs comme « Connection failed [IP: …] ».

---

## 🧾 Conclusion

Quand `apt-get update` renvoie des erreurs du type “Connection failed [IP: …]” avec une adresse IPv6, c’est très probablement lié à un **problème de connectivité IPv6** : réseau incompatible, IPv6 mal configurée, ou support absent.
La **solution la plus fiable et simple** consiste à forcer `apt` à utiliser IPv4 — soit ponctuellement, soit de façon permanente. Si tu le souhaites, tu peux aussi diagnostiquer avec les commandes ci-dessus pour confirmer le problème.

```

Si tu veux, je peux maintenant t’écrire **un script Bash** prêt à l’emploi (diagnostic + correctifs) que tu peux lancer sur ta machine pour tester et corriger automatiquement ce problème.
::contentReference[oaicite:7]{index=7}
```

[1]: https://bugs.launchpad.net/bugs/1843554?utm_source=chatgpt.com "Bug #1843554 “apt update is trying to use IPv6 for some ..."
[2]: https://askubuntu.com/questions/620317/apt-get-update-stuck-connecting-to-security-ubuntu-com?utm_source=chatgpt.com "apt-get update stuck: Connecting to security.ubuntu.com"
[3]: https://forums.raspberrypi.com/viewtopic.php?t=173052&utm_source=chatgpt.com "apt-get and ipv6 troubles"
[4]: https://askubuntu.com/questions/759524/problem-with-ipv6-sudo-apt-get-update-upgrade?utm_source=chatgpt.com "Problem with IPv6 sudo apt-get update/upgrade"
[5]: https://askubuntu.com/questions/781030/have-to-use-o-acquireforceipv4-true-to-do-apt-get-update-upgrade?utm_source=chatgpt.com "Have to use -o Acquire::ForceIPv4=true to do apt-get ..."
[6]: https://www.namehero.com/blog/how-do-i-disable-ipv6-on-ubuntu/?utm_source=chatgpt.com "How Do I Disable IPv6 On Ubuntu?"
