import numpy as np
import pyrealsense2 as rs
import cv2
import copy
import time

class D435:
    def __init__(self):

        bag = r'/home/vdr/Desktop/RealSense/test2.bag'

        self.pipeline = rs.pipeline()
        config = rs.config()

        #From a bag file
        config.enable_device_from_file(bag, False)
        config.enable_all_streams()


        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        playback = device.as_playback()
        playback.set_real_time(False)

        '''  #For real time capturing
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        self.depth_sensor = pipeline_profile.get_device().first_depth_sensor()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        '''
        self.pipeline.start(config)
    def video(self):
        align_to = rs.stream.color
        align = rs.align(align_to)
        for i in range(10):
            self.pipeline.wait_for_frames()
        while True:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            self.depth_frame = depth_frame

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            self.depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            '''
            #Manually aligning depth and color frames and visualizing them by usig opencv
            
            
            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape
    
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                                 interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))
            
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow("RGB", images) #images
            cv2.waitKey(1)
           '''


            self.depth_distance(233, 124)

            #-0.09791105, -0.01570919,  0.26800001

            color_cvt = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            self.show(color_cvt)

            break
    def depth_distance(self, x, y):
        depth_intrin = self.depth_intrin
        #ix, iy = self.ix, self.iy
        #depth_scale = self.depth_sensor.get_depth_scale()
        udist = self.depth_frame.get_distance(x, y) #in meters

        point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], udist)
        point_array = np.asanyarray(point)

        point_to_pixel = rs.rs2_project_point_to_pixel(depth_intrin, point)
        print(point)
        print(point_to_pixel)

        return point_array
        #print(point)
        #print(point_array)

    def to_point_cloud(self, start_u1, end_u2, start_v1, end_v2):
        pointcloud = []
        for width in range(start_u1, end_u2):
            for heigth in range(start_v1, end_v2):
               points = self.depth_distance(width, heigth)
               print(points)
               pointcloud.append(points)

                #pointcloud.append(self.depth_distance(width,heigth))
        #print(pointcloud)
        self.img_origin[start_v1:end_v2, start_u1:end_u2] = (0,0,255)
       # self.img_origin[start_u1:end_u2, start_v1:end_v2] = (0, 0, 255)  # 100:132, 200:241     #flipped in opencv
        update = cv2.cvtColor(self.img_origin, cv2.COLOR_BGR2RGB)
        cv2.imshow("Color stream", update)

    def show(self, img):
        self.img_origin = img
        #self.to_point_cloud(100, 132, 200, 241)
        while True:
            '''
            #Projection check

            self.img_origin[124, 233] = (0, 0, 255)  #v, u
            update = cv2.cvtColor(self.img_origin, cv2.COLOR_BGR2RGB)
            cv2.imshow("Color stream", update)
            
            
            #Testing for the following values:
            #241 132      #u,v
            # [-0.04554886743426323, -0.056116145104169846, 0.3360000252723694]  #x,y,z

            '''
            
            
            

            cv2.imshow("Color stream", self.img_origin)
            key = cv2.waitKey(10)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break






class To_bag:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        #For real time capturing
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        self.depth_sensor = pipeline_profile.get_device().first_depth_sensor()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.config.enable_record_to_file('test2.bag')
        self.pipeline.start(self.config)
    def video(self):
        for i in range(30):
            self.pipeline.wait_for_frames()
        self.pipeline.stop()

if __name__ == "__main__":
    #To_bag().video()
    D435().video()
