# Black fly + gimbal node for detection and following apriltags
[Camera used for the project](http://softwareservices.flir.com/BFS-PGE-50S5/latest/Model/spec.html)

# Preferable settings
| Name | Value | description as I understand it |
| --- | :--- | :--- |
| acquisition_frame_rate | less then 25 | user controlled frame rate in Hz |
| acquisition_frame_rate_enable | False | |
| exposure_mode | Timed | Have no idea... |
| exposure_auto | Continuous | Once / Off - if exposure time is calculated automatically |
| exposure_time | 15000.0 mcs | if auto exposure is Off, this number will be used|
| auto_exposure_time_upper_limit | 30000 mcs | Upper limit for opening shutter |
| gain_selector | All | Have no idea... |
| auto_gain | Continuous | Have no idea... |
| gain | 0.0 | Have no idea... |
| brightness | 1.7 | Black level offset |
| sharpening_enable | False | sharpening feature |
| auto_sharpness | False |  |
| sharpness | 0.0 |  | 
| sharpening_threshold |0.0 |  |
| saturation_enable | False | Depth or intensity of colour present within an image |
| saturation | 100 | | 
| gamma_enable | True | gamma correction of colors |
| gamma | 0.8 | |
| auto_white_balance | Continuous | Color shifts because of different conditions |
| white_balance_blue_ratio | 2.58 | |
| white_balance_red_ratio | 0.98 | |
| image_format_roi_width | 0 | preferred outer image width (0 means max) |
| image_format_roi_height | 0 | preferred outer image height (0 means max) |
| image_format_x_offset | 0 | roi offset in x coordinate |
| image_format_y_offset | 0 | roi offset in y coordinate |
| image_format_x_binning | 4 | how many pixels are counted as one (x coor). Not changing the resolution, but the image quality |
| image_format_y_binning | 4 | |
| image_format_x_decimation | 1 | missing of n rows repeatedly (reduce the resolution) |
| image_format_y_decimation | 1 | |
| image_format_x_reverse | False | |
| image_format_y_reverse | False | |
| image_format_color_coding | BayerRG8 | |
Other settings are not important