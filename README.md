# GF1-RS-index
- Gf-1 WFV is a 16 m resolution sensor, with 4 bands.The software can calculate multiple remote sensing indices (NDVI, OSAVI, RVI, EVI, NDWI) and surface reflectance of ROI at the same time, extract them to the .csv file, and automatically rename followed the GF-1 file naming rules.
- GF-1 WFV是我国高分一号16米分辨率WFV传感器，拥有4个波段。本软件可以实现同时对多幅影像指定的多个感兴趣区域计算多个遥感指数（NDVI, OSAVI, RVI, EVI, NDWI）以及各波段地表反射率值Ref，并提取到csv文件，并自动按照GF-1文件命名规则提取日期等信息自动批量命名输出文件。

![image](https://user-images.githubusercontent.com/44941550/167136311-0a1d866c-4f43-4515-b11e-12209cf8c941.png)

# 操作步骤
- （1）运行Anaconda中的Jupyter notebook 
- （2）创建Python3代码
- （3）准备好输入文件
将需要预处理的GF-1WFV数据文件放到一个文件夹中，路径仅有英文、数字和下划线这三种类型的字符。
- （4）将代码输入框中并修改代码
 
①在第116行源代码在这里设置统计量指标，有如下几种选择average, mean, max, min, median, sum, std, unique。

②如果不想计算该软件预设的遥感指数，可以在第126-130行源代码输入需要统计的遥感指数名称，并且在第140行之后修改计算公式，并且保证遥感指数名称前后一致。

③在第180-183行源代码输入区域shp文件路径、输入待提取tif文件夹路径、输出文件夹路径。
