# Provisioning example

Keep real deployment provisioning outside Git. Point `build-image --provision`
or `[provisioning] directory=` at a directory containing only the data that the
node should receive under `/home/mpfc/keys/`.

Typical names may include:

```text
drone.key
hivelink.key
hivelink.pubkey
hivelink_links.json
tak.pem
node.json
```

The builder copies the directory as data and does not execute anything from it.
WFB-NG adapts `/etc/drone.key` back to the canonical
`/home/mpfc/keys/drone.key` path.
