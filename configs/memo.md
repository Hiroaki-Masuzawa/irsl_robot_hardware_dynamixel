
# ドキュメントの作り方
このディレクトリで以下コマンドを打つとdocsにスキーマに関するドキュメントができる．
```
docker run --rm -v `pwd`:/schema gisaia/jsonschema2md:0.1.2 -d /schema -o /schema/docs -x -
```

- 参考
    - https://hub.docker.com/r/gisaia/jsonschema2md
        - 使用しているdocker image
        