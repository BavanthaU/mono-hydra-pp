# Kimera Loop-Closure Vocabulary

The expanded `ORBvoc.yml` file and the compressed `ORBvoc.zip` archive are
larger than GitHub's normal file-size guidance and are therefore not tracked in
the public repository.

For local runs, download and install it with:

```bash
src/mono_hydra_vio/scripts/download_orb_vocabulary.sh
```

The ITC/uHumans/7Scenes launch files expect:

```text
src/mono_hydra_vio/vocabulary/ORBvoc.yml
```

After the package has already been built, the installed helper can also populate
the package-share vocabulary directory:

```bash
source install/setup.bash
ros2 run mono_hydra_vio download_orb_vocabulary
```
