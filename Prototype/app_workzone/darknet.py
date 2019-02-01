from ctypes import *
import math
import random
import prototype.app_pedestrian.options as darknet_files
#from numpy.ctypeslib import ndpointer
from numpy import float32

## re-create the coco data file, with the right locations
with open(darknet_files.data, "wb") as datafile:
    datafile.write(
"""classes= 10
train  = /home/pjreddie/data/coco/trainvalno5k.txt
valid  = {:s}coco_testdev
#valid = {:s}data/coco_val_5k.list
names = {:s}data/coco.names
backup = {:s}backup/
eval=coco""".format(*((darknet_files.darknet_location,)*4))
                  )


def sample(probs):
    s = sum(probs)
    probs = [a/s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs)-1

def c_array(ctype, values):
    arr = (ctype*len(values))()
    arr[:] = values
    return arr

class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]

class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]

class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]

    

#lib = CDLL("/home/pjreddie/documents/darknet/libdarknet.so", RTLD_GLOBAL)
lib = CDLL(darknet_files.code, RTLD_GLOBAL)
lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

set_gpu = lib.cuda_set_device
set_gpu.argtypes = [c_int]

make_image = lib.make_image
make_image.argtypes = [c_int, c_int, c_int]
make_image.restype = IMAGE

get_network_boxes = lib.get_network_boxes
get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(c_int), c_int, POINTER(c_int)]
get_network_boxes.restype = POINTER(DETECTION)

make_network_boxes = lib.make_network_boxes
make_network_boxes.argtypes = [c_void_p]
make_network_boxes.restype = POINTER(DETECTION)

free_detections = lib.free_detections
free_detections.argtypes = [POINTER(DETECTION), c_int]

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

network_predict = lib.network_predict
network_predict.argtypes = [c_void_p, POINTER(c_float)]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

do_nms_obj = lib.do_nms_obj
do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

do_nms_sort = lib.do_nms_sort
do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

rgbgr_image = lib.rgbgr_image
rgbgr_image.argtypes = [IMAGE]

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)

def classify(net, meta, im):
    out = predict_image(net, im)
    res = []
    for i in range(meta.classes):
        res.append((meta.names[i], out[i]))
    res = sorted(res, key=lambda x: -x[1])
    return res

def detect(net, meta, c_im, image, thresh=.5, hier_thresh=.5, nms=.45):
    #im = load_image(image, 0, 0)
    h,w,c = image.shape
    image = image.transpose((2,0,1)).flatten().astype(float32)/255
    image2 = c_array(c_float, image)
    #c_im = IMAGE(w,h,c, image2)
    assert c_im.w == w and c_im.h == h
    memmove(c_im.data, image2, c*w*h)
    floatout = predict_image(net, c_im)
    num = c_int(0)
    pnum = pointer(num)
    dets = get_network_boxes(net, w, h, .4, .5, None, 0, pnum)
    num = pnum[0]
    if (nms): do_nms_obj(dets, num, meta.classes, nms);

    res = []
    for j in xrange(num):
        for i in xrange(meta.classes):
            if dets[j].prob[i] > 0:
                b = dets[j].bbox
                #res.append((meta.names[i], dets[j].prob[i], (b.x, b.y, b.w, b.h)))
                result = {'label':meta.names[i],
                          'topleft':{'x':b.x-b.w/2, 'y':b.y-b.h/2},
                          'bottomright':{'x':b.x+b.w/2, 'y':b.y+b.h/2}}
                res.append(result)
    #res = sorted(res, key=lambda x: -x[1])
    #free_image(im)
    free_detections(dets, num)
    return res
    
def load(h,w):
    net = load_net(darknet_files.cfg, darknet_files.weights, 0)
    meta = load_meta(darknet_files.data)
    c_im = IMAGE(w, h, 3, (c_float*(w*h*3))())
    return net, meta, c_im
    
if __name__ == "__main__":
    from cv2 import imread
    import time
    im = imread(darknet_files.darknet_location+"data/dog.jpg")
    net, meta, c_im = load(im.shape[0], im.shape[1])
    im = im[:,:,::-1]
    r = detect(net, meta, c_im, im)
    for result in r: print result
