# Fixed code version

**Release:** [mono_hydrapp_v1.0](https://github.com/BavanthaU/mono-hydra-pp/releases/tag/mono_hydrapp_v1.0)

Use this release to retrieve the updated code associated with the ISPRS paper
submission. Later development continues on `main`; new snapshots receive new
tags. The release contains the source, configuration, documentation, and journal
figures. Its Git commit and tree IDs are recorded in `release-manifest.json`
on the release page.

## Checkout

Create a new workspace and clone the fixed version:

```bash
mkdir -p ~/mono_hydra_paper_ws
cd ~/mono_hydra_paper_ws
git clone --branch mono_hydrapp_v1.0 --single-branch \
  https://github.com/BavanthaU/mono-hydra-pp.git src
git -C src rev-parse HEAD
```

Compare the printed commit with `source_commit` in the release manifest.
A detached HEAD is expected when cloning a tag. Follow the
[build instructions](README.md#build) and [ITC run guide](ITC_REVISION.md).
Run their commands from this workspace root.

For an existing clean clone:

```bash
git fetch origin tag mono_hydrapp_v1.0
git switch --detach mono_hydrapp_v1.0
```

## Downloads and verification

The release assets include a source archive, `release-manifest.json`, and
`SHA256SUMS`. Download all three into one directory and verify them:

```bash
sha256sum -c SHA256SUMS
```

The checksums refer to the named release assets. GitHub's automatically
generated ZIP and tarball downloads are separate files.

## Models and runtime inputs

- [M2H-MX checkpoints](https://huggingface.co/Bavantha11/m2h-mx)
- [ONNX exports](https://huggingface.co/Bavantha11/vio-stack-jetson-weights)
- [Pinned model revisions and file hashes](reproducibility/model-manifest.json)

Choose **ScanNet** or **NYUDv2** weights for your scene and select the matching
`perception_dataset`. Record that choice with your experiment settings.

Use the commit revisions in the model manifest when downloading from Hugging
Face. For example, the published M2H-MX checkpoint collection is pinned at
`a3f1cc6b6fd2b136d3c38780344b2fa8af43dc54`, and the ONNX collection at
`0de285a1ea8c278d9822537c484c386bea8fee81`.

The local ITC checkpoint and SuperPoint export are not present in those Hub
snapshots. Their hashes, together with the ORB vocabulary hash, are recorded in
[itc-artifacts.sha256](reproducibility/itc-artifacts.sha256). Obtain the matching
files for the default ITC setup, then verify from the repository root:

```bash
sha256sum -c reproducibility/itc-artifacts.sha256
```

The fixed code version uses RVIO2–SuperPoint as the main ITC frontend, with PTC
off by default. The [ITC guide](ITC_REVISION.md) records the run settings and
on/off commands. Matching results also requires the same input bag, calibration,
model artifacts, and runtime dependencies. This is a source release, not a
container image or a complete experiment archive.

## Validation and citation

The implementation matches commit
`5a7ef2fc47d64c2839e062d3258c95d0506d87ed`; release preparation adds documentation
and artifact manifests. Prior checks passed 17 Python tests, 19 Hydra tests,
12 launch combinations, and the builds listed in the [ITC guide](ITC_REVISION.md#validation-scope).
The full dataset experiments were not rerun during release preparation.

In reproducibility notes, record the release tag, the full source commit from
the manifest, and the model hashes. Cite the relevant
[papers](README.md#bibliography-and-citation).
