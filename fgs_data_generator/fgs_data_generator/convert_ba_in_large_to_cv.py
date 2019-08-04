from sys import exit, argv


# See this link for detail
# http://grail.cs.washington.edu/projects/bal/
def main(args=None):
    with open(args[-1], 'r') as f:
        all_line = f.readlines()
        index = 0
        all_data = []
        all_observations = []
        all_camera_params = []
        all_points = []
        num_cameras = 0
        num_points = 0
        num_observations = 0
        num_point_params = 3
        num_camera_param = 9
        point = []
        camera_param = []
        for line in all_line:
            values_str = line.split(' ')
            values = [float(value_str) for value_str in values_str if value_str != '']

            if index == 0:
                # <num_cameras> <num_points> <num_observations>
                num_cameras = values[0]
                num_points = values[1]
                num_observations = values[2]
                print('num_camera      : {}'.format(num_cameras))
                print('num_observations: {}'.format(num_observations))
                print('num_points      : {}'.format(num_points))
            elif index < num_observations + 1:
                # <camera_index_num_observations>
                # <point_index_num_observations>
                # <x_num_observations> <y_num_observations>
                all_observations.append(values)
            elif index < num_camera_param * num_cameras + num_observations + 1:
                camera_param.append(values[0])
                if (len(camera_param) == num_camera_param):
                    all_camera_params.append(camera_param)
                    camera_param = []
            else:
                point.append(values[0])
                if (len(point) == num_point_params):
                    all_points.append(point)
                    point = []

            index += 1

        all_data.append(all_observations)
        all_data.append(all_camera_params)
        all_data.append(all_points)

    file_handle = cv2.FileStorage('/tmp/cv_ba_in_large.yaml', cv2.FileStorage_WRITE)
    file_handle.write('observations', np.array(all_data[0]))
    file_handle.write('camera_parameters', np.array(all_data[1]))
    file_handle.write('points', np.array(all_data[2]))

    print('done')


if __name__ == '__main__':
    main()
