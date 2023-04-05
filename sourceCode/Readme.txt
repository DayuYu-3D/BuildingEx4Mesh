1. 使用C++17标准
2. 使用了filesystem，mingw编译器低于9版本无法编译；或者使用vs2019编译器
3. 使用vcpkg安装cgal
4. 2021年5月份时，最新的vcpkg用bug，即无法安装yasm-tool64位版本，使用以下解决：
使用5月份之前的vcpkg版本
5. 读取jpg类型的纹理图片采用的是Qimage读取，若显示没有jpg插件，
则需要将Qt/plugins/imageFormats中的动态库配置环境或者拷贝到目录中
