import numpy
from osgeo import gdal,osr,ogr
import shapefile
import sys
from pandas.core.frame import DataFrame
import os
def search_f(inpath):
    # 检索待处理影像
    F_list = []
    for root,dir,fields in os.walk(inpath):
        for field in fields:
            if field.endswith("tif") is True:
                f_fn = os.path.join(inpath, field)
                F_list.append(f_fn)
    return F_list
def Getshp_FID(shp_fn):
    """获取要素FID、ROI字段名称"""
    # 用pyshp库打开矢量
    file = shapefile.Reader(shp_fn)
    # 获取要素属性、要素数量
    features = file.records()
    features_number = len(file.records())
    # 获取要素FID、ROI名称列表
    FID_list = []
    Feature_name_list = []
    for i in range(features_number):
        feature = str(features[i])
        # 从Feature中获取FID
        feature_id = int(feature.split("#")[1].split(":")[0])
        FID_list.append(feature_id)
        # 从Feature中获取FID对应的数字
        feature_name = eval(str(features[i]).split(":")[1].split("[")[1].split("]")[0])
        Feature_name_list.append(feature_name)
    return FID_list, Feature_name_list, features_number
def zonal_stats(feat, input_zone_polygon, raster_paths):
    """统计shp图中要素反射率"""
    # 创建空列表，用于存储计算后的反射率值
    zonal_average = []
    # 打开影像
    raster = gdal.Open(raster_paths)
    # 打开图层
    shp = ogr.Open(input_zone_polygon)
    lyr = shp.GetLayer()
    # Get raster georeference info
    transform = raster.GetGeoTransform()
    band = raster.RasterCount
    # 左上角坐标；影像分辨率
    xOrigin = transform[0]
    yOrigin = transform[3]
    pixelWidth = transform[1]
    pixelHeight = transform[5]
    #矢量重投影为与栅格相同的投影，CoordinateTransformation 坐标转换
    sourceSR = lyr.GetSpatialRef()
    targetSR = osr.SpatialReference()
    targetSR.ImportFromWkt(raster.GetProjectionRef())
    coordTrans = osr.CoordinateTransformation(sourceSR, targetSR)
    # feat = lyr.GetNextFeature()
    geom = feat.GetGeometryRef()
    geom.Transform(coordTrans)
    # Get extent of feat
    geom = feat.GetGeometryRef()
    if (geom.GetGeometryName() == "MULTIPOLYGON"):
        count = 0
        pointsX = []
        pointsY = []
        for polygon in geom:
            geomInner = geom.GetGeometryRef(count)
            ring = geomInner.GetGeometryRef(0)
            numpoints = ring.GetPointCount()
            for p in range(numpoints):
                lon, lat, z = ring.GetPoint(p)
                pointsX.append(lon)
                pointsY.append(lat)
            count += 1
    elif (geom.GetGeometryName() == 'POLYGON'):
        ring = geom.GetGeometryRef(0)
        numpoints = ring.GetPointCount()
        pointsX = []
        pointsY = []
        for p in range(numpoints):
            lon, lat, z = ring.GetPoint(p)
            pointsX.append(lon)
            pointsY.append(lat)
    else:
        sys.exit("ERROR: Geometry needs to be either Polygon or Multipolygon")
    xmin = min(pointsX)
    xmax = max(pointsX)
    ymin = min(pointsY)
    ymax = max(pointsY)
    # Specify offset and rows and columns to read
    xoff = int((xmin - xOrigin) / pixelWidth)
    yoff = int((yOrigin - ymax) / pixelWidth)
    xcount = int((xmax - xmin) / pixelWidth) + 1
    ycount = int((ymax - ymin) / pixelWidth) + 1
    # Create memory target raster
    target_ds = gdal.GetDriverByName("MEM").Create('', xcount, ycount, 5, gdal.GDT_Byte) #gdal.GDT_Byte 取决于元数据里面的像元值格式band的type
    target_ds.SetGeoTransform((xmin, pixelWidth, 0, ymax, 0, pixelHeight,))

    # Create for target raster the same projection as for the value raster
    raster_srs = osr.SpatialReference()
    raster_srs.ImportFromWkt(raster.GetProjectionRef())
    target_ds.SetProjection(raster_srs.ExportToWkt())
    # Rasterize zone polygon to raster
    raster_bandlist = list(range(band, 0, -1))
    b_v = [1]*band
    gdal.RasterizeLayer(target_ds, raster_bandlist, lyr, burn_values=b_v)
    for i in range(band):
        banddataraster = raster.GetRasterBand(i + 1)
        dataraster = banddataraster.ReadAsArray(xoff, yoff, xcount, ycount).astype(numpy.float)
        bandmask = target_ds.GetRasterBand(i + 1)
        datamask = bandmask.ReadAsArray(0, 0, xcount, ycount).astype(numpy.float)
        # Mask zone of raster
        zoneraster = numpy.ma.masked_array(dataraster, numpy.logical_not(datamask))
        # Calculate statistics of zonal raster
        zonal_average.append(numpy.average(zoneraster))
    return zonal_average
def get_vi_ref(input_zone_polygon, raster_paths, Ref_out, VI_out):
    """统计矢量范围均值"""
    # 第一步：获取要素FID和对应ROI的列表
    FID_list, Feature_name_list, features_number = Getshp_FID(input_zone_polygon)
    shp = ogr.Open(input_zone_polygon)
    lyr = shp.GetLayer()
    # 第二步：依次获取反射率Ref、NDIV和OSAVI
    Ref = []
    NDVI = []
    OSAVI = []
    RVI = []
    EVI = []
    NDWI = []
    for FID in FID_list:
        feat = lyr.GetFeature(FID)
        Ref_meanValue = zonal_stats(feat, input_zone_polygon, raster_paths)
        # 获取各波段反射率值
        b1 = Ref_meanValue[0]  # blue
        b2 = Ref_meanValue[1]  # green
        b3 = Ref_meanValue[2]  # red
        b4 = Ref_meanValue[3]  # nir
        # 计算植被指数
        ndvi = (float(b4) - b3) / (float(b4) + b3)
        osavi = 1.16 * (b4 - b2) / (b4 + b2 + 0.16)
        rvi = float(b4) / b3
        evi = 2.5*(float(b4)-b3)/(b4+6.0*b3-7.5*b1+1)
        ndwi = (float(b2) - b4) / (float(b2) + b4)
        # 追加至列表
        Ref.append(Ref_meanValue)
        NDVI.append(ndvi)
        OSAVI.append(osavi)
        RVI.append(rvi)
        EVI.append(evi)
        NDWI.append(ndwi)
    NDVI = numpy.array(NDVI)
    OSAVI = numpy.array(OSAVI)
    RVI = numpy.array(RVI)
    EVI = numpy.array(EVI)
    NDWI = numpy.array(NDWI)   
    # NDVI、OSAVI数组纵向合并，把VI这句删掉，看看能否不算总体平均
    VI = numpy.stack((NDVI, OSAVI, RVI, EVI,NDWI)) #可以变换为hstack 和 stack
# 第三步：Ref VI 输出dataframe
VI = DataFrame(VI, columns=FID_list, index=["NDVI", "OSAVI","RVI",'EVI',"NDWI"])
    Ref = DataFrame(Ref, columns=['blue', 'green', 'red', 'nir'], index=FID_list)
    # Ref VI 输出为CSV
    Ref.to_csv(Ref_out)
    VI.to_csv(VI_out)
    print("提取完成，小区数量为：", features_number)
    return
def main(input_zone_polygon,Ref_vi_out,raster_path):
    fn_list =  search_f(raster_path)
    for f in fn_list:
        print("提取", f)
        # 以影像日期为csv名
        Ref = f.split("\\")[-1].split(".")[2].split("_")[1] + f.split("\\")[-1].split(".")[2].split("_")[2] + "_ref.csv"
        VI  = f.split("\\")[-1].split(".")[2].split("_")[1] + f.split("\\")[-1].split(".")[2].split("_")[2] + "_vi.csv"
        Ref_path = os.path.join(Ref_vi_out, Ref) 
        VI_path = os.path.join(Ref_vi_out, VI)
        get_vi_ref(input_zone_polygon, f, Ref_path, VI_path)
if __name__ == '__main__':
    input_zone_polygon = r"D:\3Sproject\taskdata\2018\20180511task\shp\xylakeCopy.shp"#研究区shp图
    Ref_vi_out = r"D:\3Sproject\3Sdata\GF1\result"#vi和ref输出路径
    raster_path = r"D:\3Sproject\3Sdata\GF1\RSDgeotrue"#待提取影像路径
    main(input_zone_polygon, Ref_vi_out, raster_path)