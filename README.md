# popo
project "popo" will be make the history.

"ぽぽ"は「メモリを犠牲に、早く、美しく」をメインテーマに制作されました。
異次元の速度で描画される現実世界をお楽しみください。

# 環境構築
ここではWindows環境での実行を前提としています。
## Qt 6.2.4以上をインストールする
https://www.qt.io/download-dev
## CMahe GUIをインストールする
https://cmake.org/download/
必ずプレリリースではなく正式リリース版をインストールしてください。
##  サードパーティーをvcpkgを利用してインストールする。
### vcpkgがインストールされていない場合はインストールする。
#### コマンド
``C:\``に移動し管理者権限で以下のコマンドを実行する。
```.bat
git clone https://github.com/microsoft/vcpkg
cd c:\vcpkg
git pull
.\bootstrap-vcpkg.bat
cd c:\
c:\vcpkg\vcpkg update
c:\vcpkg\vcpkg integrate install 
```
#### 環境変数
システムの環境変数に以下の環境変数を追加してください。t環境変数を設定した後は必ず一度シャットダウンしてください。
| 環境変数名             | パス                                                |
|------------------------|----------------------------------------------------|
| VCPKG_ROOT             | c:\vcpkg                                           |
| VCPKG_DEFAULT_TRIPLET  | x64-windows                                        |
| CMAKE_TOOLCHAIN_FILE   | c:/vcpkg/scripts/buildsystems/vcpkg.cmake          |
| LIB                    | c:/vcpkg/installed/x64-windows/lib                 |
| INCLUDE                | c:/vcpkg/installed/x64-windows/include             |
| Path                   | c:/vcpkg                                           |
| Path                   | c:/vcpkg/installed/x64-windows/bin                 |

```.bat
powershell -command "[System.Environment]::SetEnvironmentVariable(\"VCPKG_ROOT\", \"c:\vcpkg\", \"Machine\")"
powershell -command "[System.Environment]::SetEnvironmentVariable(\"VCPKG_DEFAULT_TRIPLET\", \"x64-windows\", \"Machine\")"
powershell -command "[System.Environment]::SetEnvironmentVariable(\"CMAKE_TOOLCHAIN_FILE\", \"c:/vcpkg/scripts/buildsystems/vcpkg.cmake\", \"Machine\")"
powershell -command "$oldpath = [System.Environment]::GetEnvironmentVariable(\"LIB\", \"Machine\"); $oldpath += \";c:\vcpkg\installed\x64-windows\lib\"; [System.Environment]::SetEnvironmentVariable(\"LIB\", $oldpath, \"Machine\")"
powershell -command "$oldpath = [System.Environment]::GetEnvironmentVariable(\"INCLUDE\", \"Machine\"); $oldpath += \";c:\vcpkg\installed\x64-windows\include\"; [System.Environment]::SetEnvironmentVariable(\"INCLUDE\", $oldpath, \"Machine\")"
powershell -command "$oldpath = [System.Environment]::GetEnvironmentVariable(\"Path\", \"Machine\"); $oldpath += \";c:\vcpkg\"; [System.Environment]::SetEnvironmentVariable(\"Path\", $oldpath, \"Machine\")"
powershell -command "$oldpath = [System.Environment]::GetEnvironmentVariable(\"Path\", \"Machine\"); $oldpath += \";c:\vcpkg\installed\x64-windows\bin\"; [System.Environment]::SetEnvironmentVariable(\"Path\", $oldpath, \"Machine\")"
```

### ライブラリをインストールする
| ライブラリ名            | コマンド                                   |
|------------------------|-------------------------------------------|
| SDL3                   | vcpkg install sdl3                        |

## PCLをAll in one installerを使用してインストールする
All-in-One Installerでインストールします。
以下のリンクよりバージョン1.13.1(最新のバージョンは1.15.1ですが実行できません)

https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.13.1

ダウウンロードしたらインストールを実行してください。展開先は推奨される``Program File``内で構いません。
### 環境変数の設定
システムの環境変数に以下の環境変数を追加してください。追加した後、PCの再起動を行ってください。
| 環境変数名              | パス                                      |
|------------------------|--------------------------------------------|
| Path                   | C:\Program Files\OpenNI2                  |
|                        | C:\Program Files\PCL 1.13.1\bin           |
|                        | C:\Program Files\PCL 1.13.1\3rdParty      |
|                        | C:\Program Files\PCL 1.13.1\3rdParty\VTK\bin |
|                        | C:\Program Files\OpenNI2\Tools            |
| PCL_ROOT               | C:\Program Files\PCL 1.13.1               |
| OPENNI2_INCLUDE64      | C:\Program Files\OpenNI2\Include\         |
| OPENNI2_LIB64          | C:\Program Files\OpenNI2\Lib\             |
| OPENNI2_REDIST64       | C:\Program Files\OpenNI2\Redist\          |

```.bat
powershell -command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';C:\Program Files\OpenNI2', 'Machine')"
powershell -command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';C:\Program Files\PCL 1.13.1\bin', 'Machine')"
powershell -command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';C:\Program Files\PCL 1.13.1\3rdParty', 'Machine')"
powershell -command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';C:\Program Files\PCL 1.13.1\3rdParty\VTK\bin', 'Machine')"
powershell -command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';C:\Program Files\OpenNI2\Tools', 'Machine')"
powershell -command "[Environment]::SetEnvironmentVariable('PCL_ROOT', 'C:\Program Files\PCL 1.13.1', 'Machine')"
powershell -command "[Environment]::SetEnvironmentVariable('OPENNI2_INCLUDE64', 'C:\Program Files\OpenNI2\Include\', 'Machine')"
powershell -command "[Environment]::SetEnvironmentVariable('OPENNI2_LIB64', 'C:\Program Files\OpenNI2\Lib\', 'Machine')"
powershell -command "[Environment]::SetEnvironmentVariable('OPENNI2_REDIST64', 'C:\Program Files\OpenNI2\Redist\', 'Machine')"
```
