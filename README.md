# Hongo (ミクロシミュレーション)

## Docker の準備

### Windows の場合

Windows では WSL 2 が有効化されている必要があります。
コマンドプロンプトや PowerShell で以下のコマンドを実行し、WSL 2 を有効化してください。

```powershell
wsl --install
```

### macOS & Windows 共通

以下から Docker Desktop をダウンロード。

https://www.docker.com/products/docker-desktop/

インストール後に再起動やログインを促された場合は、指示に従ってください。
アカウント作成は[ここ](https://hub.docker.com/signup)から行えます。

## ビルド & 実行

以下のコマンドを実行することにより、Hongo (ミクロシミュレーションプログラム) を実行することができます。
初回の実行はイメージのダウンロードやビルドに数分〜数十分かかります。Wi-Fi 環境での実行を推奨します。

```bash
docker compose up
```

## 入力/出力ファイル指定

デフォルトでは以下のフォルダが入力/出力フォルダとして指定されています。

`.env` の `minimum` の部分を `shibuya` などに変更することで、入力/出力フォルダを指定することができます。
Mac の場合は不可視ファイルのため、Finder でデフォルトでは表示されないので、VSCode などのエディタで編集してください。

```env.txt
INPUT=./input/minimum
OUTPUT=./output/minimum
```
