## 写在前面

非常高兴能和各位坎巴拉的玩家在这里见面！本程序能够实现**赤道平面近坎轨道**返回**坎巴拉太空中心**，我非常期待能和大家在这里交流，最终能够使得这个程序变得更好！

## 使用说明

### 特别注意

请根据你的飞船质量和迎风面积对下列文件中的代码进行修改

- chose_exchange_point.py
- second_landing_perdict.py


```
C_D, C_L, m, A = 2.0, 0.0, 4132, np.pi*1.3*1.3  # 阻力系数、升力系数、质量、面积
```


### 环境要求

- 安装**坎巴拉太空计划**并安装了[KPRC](https://www.curseforge.com/kerbal/ksp-mods/krpc-control-the-game-using-c-c-java-lua-python/files)
- Python 3.8 及以上
- 安装 `krpc` 库：`pip install krpc`

### KPRC 相关推荐资料

- [官方文档](https://krpc.github.io/krpc/)
- [基础使用指南](https://www.bilibili.com/opus/400594077896210025)


### 运行程序

在终端中运行以下命令启动回收程序：
```
python main.py
```

## 当前程序存在的问题

- 落点精度不太高
- 飞船再入段攻角较大


### 联系方式

如有任何问题，请通过以下方式联系我：

- 邮箱: 1405988307@qq.com
- GitHub: [sjf2001](https://github.com/sfj2001)

感谢使用本程序！

