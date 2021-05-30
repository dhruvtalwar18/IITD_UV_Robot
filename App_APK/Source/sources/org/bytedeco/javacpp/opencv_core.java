package org.bytedeco.javacpp;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.nio.ShortBuffer;
import org.bytedeco.javacpp.annotation.ByPtrPtr;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Convention;
import org.bytedeco.javacpp.annotation.Index;
import org.bytedeco.javacpp.annotation.MemberGetter;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.Opaque;
import org.bytedeco.javacpp.annotation.StdString;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.annotation.ValueSetter;
import org.bytedeco.javacpp.annotation.Virtual;
import org.bytedeco.javacpp.helper.opencv_core;
import org.bytedeco.javacpp.helper.opencv_imgproc;
import org.bytedeco.javacpp.presets.opencv_core;
import org.xbill.DNS.TTL;

public class opencv_core extends org.bytedeco.javacpp.helper.opencv_core {
    public static final int ACCESS_FAST = 67108864;
    public static final int ACCESS_MASK = 50331648;
    public static final int ACCESS_READ = 16777216;
    public static final int ACCESS_RW = 50331648;
    public static final int ACCESS_WRITE = 33554432;
    public static final int ALGORITHM = 6;
    public static final int BOOLEAN = 1;
    public static final int BORDER_CONSTANT = 0;
    public static final int BORDER_DEFAULT = 4;
    public static final int BORDER_ISOLATED = 16;
    public static final int BORDER_REFLECT = 2;
    public static final int BORDER_REFLECT101 = 4;
    public static final int BORDER_REFLECT_101 = 4;
    public static final int BORDER_REPLICATE = 1;
    public static final int BORDER_TRANSPARENT = 5;
    public static final int BORDER_WRAP = 3;
    public static final int BadAlign = -21;
    public static final int BadAlphaChannel = -18;
    public static final int BadCOI = -24;
    public static final int BadCallBack = -22;
    public static final int BadDataPtr = -12;
    public static final int BadDepth = -17;
    public static final int BadImageSize = -10;
    public static final int BadModelOrChSeq = -14;
    public static final int BadNumChannel1U = -16;
    public static final int BadNumChannels = -15;
    public static final int BadOffset = -11;
    public static final int BadOrder = -19;
    public static final int BadOrigin = -20;
    public static final int BadROISize = -25;
    public static final int BadStep = -13;
    public static final int BadTileSize = -23;
    public static final int CMP_EQ = 0;
    public static final int CMP_GE = 2;
    public static final int CMP_GT = 1;
    public static final int CMP_LE = 4;
    public static final int CMP_LT = 3;
    public static final int CMP_NE = 5;
    public static final int COVAR_COLS = 16;
    public static final int COVAR_NORMAL = 1;
    public static final int COVAR_ROWS = 8;
    public static final int COVAR_SCALE = 4;
    public static final int COVAR_SCRAMBLED = 0;
    public static final int COVAR_USE_AVG = 2;
    public static final int CPU_AVX = 10;
    public static final int CPU_AVX2 = 11;
    public static final int CPU_AVX512_SKX = 256;
    public static final int CPU_AVX_512BW = 14;
    public static final int CPU_AVX_512CD = 15;
    public static final int CPU_AVX_512DQ = 16;
    public static final int CPU_AVX_512ER = 17;
    public static final int CPU_AVX_512F = 13;
    public static final int CPU_AVX_512IFMA = 18;
    public static final int CPU_AVX_512IFMA512 = 18;
    public static final int CPU_AVX_512PF = 19;
    public static final int CPU_AVX_512VBMI = 20;
    public static final int CPU_AVX_512VL = 21;
    public static final int CPU_FMA3 = 12;
    public static final int CPU_FP16 = 9;
    public static final int CPU_MAX_FEATURE = 512;
    public static final int CPU_MMX = 1;
    public static final int CPU_NEON = 100;
    public static final int CPU_POPCNT = 8;
    public static final int CPU_SSE = 2;
    public static final int CPU_SSE2 = 3;
    public static final int CPU_SSE3 = 4;
    public static final int CPU_SSE4_1 = 6;
    public static final int CPU_SSE4_2 = 7;
    public static final int CPU_SSSE3 = 5;
    public static final int CPU_VSX = 200;
    public static final int CPU_VSX3 = 201;
    public static final int CV_16F = 7;
    public static final int CV_16FC1 = CV_16FC1();
    public static final int CV_16FC2 = CV_16FC2();
    public static final int CV_16FC3 = CV_16FC3();
    public static final int CV_16FC4 = CV_16FC4();
    public static final int CV_16S = 3;
    public static final int CV_16SC1 = CV_MAKETYPE(3, 1);
    public static final int CV_16SC2 = CV_MAKETYPE(3, 2);
    public static final int CV_16SC3 = CV_MAKETYPE(3, 3);
    public static final int CV_16SC4 = CV_MAKETYPE(3, 4);
    public static final int CV_16U = 2;
    public static final int CV_16UC1 = CV_MAKETYPE(2, 1);
    public static final int CV_16UC2 = CV_MAKETYPE(2, 2);
    public static final int CV_16UC3 = CV_MAKETYPE(2, 3);
    public static final int CV_16UC4 = CV_MAKETYPE(2, 4);
    public static final double CV_2PI = 6.283185307179586d;
    public static final int CV_32F = 5;
    public static final int CV_32FC1 = CV_MAKETYPE(5, 1);
    public static final int CV_32FC2 = CV_MAKETYPE(5, 2);
    public static final int CV_32FC3 = CV_MAKETYPE(5, 3);
    public static final int CV_32FC4 = CV_MAKETYPE(5, 4);
    public static final int CV_32S = 4;
    public static final int CV_32SC1 = CV_MAKETYPE(4, 1);
    public static final int CV_32SC2 = CV_MAKETYPE(4, 2);
    public static final int CV_32SC3 = CV_MAKETYPE(4, 3);
    public static final int CV_32SC4 = CV_MAKETYPE(4, 4);
    public static final int CV_64F = 6;
    public static final int CV_64FC1 = CV_MAKETYPE(6, 1);
    public static final int CV_64FC2 = CV_MAKETYPE(6, 2);
    public static final int CV_64FC3 = CV_MAKETYPE(6, 3);
    public static final int CV_64FC4 = CV_MAKETYPE(6, 4);
    public static final int CV_8S = 1;
    public static final int CV_8SC1 = CV_MAKETYPE(1, 1);
    public static final int CV_8SC2 = CV_MAKETYPE(1, 2);
    public static final int CV_8SC3 = CV_MAKETYPE(1, 3);
    public static final int CV_8SC4 = CV_MAKETYPE(1, 4);
    public static final int CV_8U = 0;
    public static final int CV_8UC1 = CV_MAKETYPE(0, 1);
    public static final int CV_8UC2 = CV_MAKETYPE(0, 2);
    public static final int CV_8UC3 = CV_MAKETYPE(0, 3);
    public static final int CV_8UC4 = CV_MAKETYPE(0, 4);
    public static final int CV_AUTOSTEP = Integer.MAX_VALUE;
    public static final int CV_AUTO_STEP = Integer.MAX_VALUE;
    public static final int CV_BACK = 0;
    public static final int CV_BadAlign = -21;
    public static final int CV_BadAlphaChannel = -18;
    public static final int CV_BadCOI = -24;
    public static final int CV_BadCallBack = -22;
    public static final int CV_BadDataPtr = -12;
    public static final int CV_BadDepth = -17;
    public static final int CV_BadImageSize = -10;
    public static final int CV_BadModelOrChSeq = -14;
    public static final int CV_BadNumChannel1U = -16;
    public static final int CV_BadNumChannels = -15;
    public static final int CV_BadOffset = -11;
    public static final int CV_BadOrder = -19;
    public static final int CV_BadOrigin = -20;
    public static final int CV_BadROISize = -25;
    public static final int CV_BadStep = -13;
    public static final int CV_BadTileSize = -23;
    public static final int CV_C = 1;
    public static final int CV_CHECK_QUIET = 2;
    public static final int CV_CHECK_RANGE = 1;
    public static final int CV_CHOLESKY = 3;
    public static final int CV_CMP_EQ = 0;
    public static final int CV_CMP_GE = 2;
    public static final int CV_CMP_GT = 1;
    public static final int CV_CMP_LE = 4;
    public static final int CV_CMP_LT = 3;
    public static final int CV_CMP_NE = 5;
    public static final int CV_CN_MAX = 512;
    public static final int CV_CN_SHIFT = 3;
    public static final int CV_COVAR_COLS = 16;
    public static final int CV_COVAR_NORMAL = 1;
    public static final int CV_COVAR_ROWS = 8;
    public static final int CV_COVAR_SCALE = 4;
    public static final int CV_COVAR_SCRAMBLED = 0;
    public static final int CV_COVAR_USE_AVG = 2;
    public static final int CV_CPU_AVX = 10;
    public static final int CV_CPU_AVX2 = 11;
    public static final int CV_CPU_AVX512_SKX = 256;
    public static final int CV_CPU_AVX_512BW = 14;
    public static final int CV_CPU_AVX_512CD = 15;
    public static final int CV_CPU_AVX_512DQ = 16;
    public static final int CV_CPU_AVX_512ER = 17;
    public static final int CV_CPU_AVX_512F = 13;
    public static final int CV_CPU_AVX_512IFMA = 18;
    public static final int CV_CPU_AVX_512IFMA512 = 18;
    public static final int CV_CPU_AVX_512PF = 19;
    public static final int CV_CPU_AVX_512VBMI = 20;
    public static final int CV_CPU_AVX_512VL = 21;
    public static final int CV_CPU_FMA3 = 12;
    public static final int CV_CPU_FP16 = 9;
    public static final int CV_CPU_MMX = 1;
    public static final int CV_CPU_NEON = 100;
    public static final int CV_CPU_NONE = 0;
    public static final int CV_CPU_POPCNT = 8;
    public static final int CV_CPU_SSE = 2;
    public static final int CV_CPU_SSE2 = 3;
    public static final int CV_CPU_SSE3 = 4;
    public static final int CV_CPU_SSE4_1 = 6;
    public static final int CV_CPU_SSE4_2 = 7;
    public static final int CV_CPU_SSSE3 = 5;
    public static final int CV_CPU_VSX = 200;
    public static final int CV_CPU_VSX3 = 201;
    public static final int CV_CXX_MOVE_SEMANTICS = 1;
    public static final int CV_CXX_STD_ARRAY = 1;
    public static final int CV_DEPTH_MAX = 8;
    public static final int CV_DIFF = 16;
    public static final int CV_DIFF_C = 17;
    public static final int CV_DIFF_L1 = 18;
    public static final int CV_DIFF_L2 = 20;
    public static final int CV_DXT_FORWARD = 0;
    public static final int CV_DXT_INVERSE = 1;
    public static final int CV_DXT_INVERSE_SCALE = 3;
    public static final int CV_DXT_INV_SCALE = 3;
    public static final int CV_DXT_MUL_CONJ = 8;
    public static final int CV_DXT_ROWS = 4;
    public static final int CV_DXT_SCALE = 2;
    public static final int CV_ErrModeLeaf = 0;
    public static final int CV_ErrModeParent = 1;
    public static final int CV_ErrModeSilent = 2;
    public static final int CV_FP16_TYPE = CV_FP16_TYPE();
    public static final int CV_FRONT = 1;
    public static final int CV_GEMM_A_T = 1;
    public static final int CV_GEMM_B_T = 2;
    public static final int CV_GEMM_C_T = 4;
    public static final int CV_GRAPH = 4096;
    public static final int CV_GRAPH_ALL_ITEMS = -1;
    public static final int CV_GRAPH_ANY_EDGE = 30;
    public static final int CV_GRAPH_BACKTRACKING = 64;
    public static final int CV_GRAPH_BACK_EDGE = 4;
    public static final int CV_GRAPH_CROSS_EDGE = 16;
    public static final int CV_GRAPH_FLAG_ORIENTED = 16384;
    public static final int CV_GRAPH_FORWARD_EDGE = 8;
    public static final int CV_GRAPH_FORWARD_EDGE_FLAG = 268435456;
    public static final int CV_GRAPH_ITEM_VISITED_FLAG = 1073741824;
    public static final int CV_GRAPH_NEW_TREE = 32;
    public static final int CV_GRAPH_OVER = -1;
    public static final int CV_GRAPH_SEARCH_TREE_NODE_FLAG = 536870912;
    public static final int CV_GRAPH_TREE_EDGE = 2;
    public static final int CV_GRAPH_VERTEX = 1;
    public static final int CV_GpuApiCallError = -217;
    public static final int CV_GpuNotSupported = -216;
    public static final int CV_HAL_BORDER_CONSTANT = 0;
    public static final int CV_HAL_BORDER_ISOLATED = 16;
    public static final int CV_HAL_BORDER_REFLECT = 2;
    public static final int CV_HAL_BORDER_REFLECT_101 = 4;
    public static final int CV_HAL_BORDER_REPLICATE = 1;
    public static final int CV_HAL_BORDER_TRANSPARENT = 5;
    public static final int CV_HAL_BORDER_WRAP = 3;
    public static final int CV_HAL_CMP_EQ = 0;
    public static final int CV_HAL_CMP_GE = 2;
    public static final int CV_HAL_CMP_GT = 1;
    public static final int CV_HAL_CMP_LE = 4;
    public static final int CV_HAL_CMP_LT = 3;
    public static final int CV_HAL_CMP_NE = 5;
    public static final int CV_HAL_DFT_COMPLEX_OUTPUT = 16;
    public static final int CV_HAL_DFT_INVERSE = 1;
    public static final int CV_HAL_DFT_IS_CONTINUOUS = 512;
    public static final int CV_HAL_DFT_IS_INPLACE = 1024;
    public static final int CV_HAL_DFT_REAL_OUTPUT = 32;
    public static final int CV_HAL_DFT_ROWS = 4;
    public static final int CV_HAL_DFT_SCALE = 2;
    public static final int CV_HAL_DFT_STAGE_COLS = 128;
    public static final int CV_HAL_DFT_TWO_STAGE = 64;
    public static final int CV_HAL_ERROR_NOT_IMPLEMENTED = 1;
    public static final int CV_HAL_ERROR_OK = 0;
    public static final int CV_HAL_ERROR_UNKNOWN = -1;
    public static final int CV_HAL_GEMM_1_T = 1;
    public static final int CV_HAL_GEMM_2_T = 2;
    public static final int CV_HAL_GEMM_3_T = 4;
    public static final int CV_HAL_SVD_FULL_UV = 8;
    public static final int CV_HAL_SVD_MODIFY_A = 4;
    public static final int CV_HAL_SVD_NO_UV = 1;
    public static final int CV_HAL_SVD_SHORT_UV = 2;
    public static final int CV_HARDWARE_MAX_FEATURE = 512;
    public static final int CV_HIST_ARRAY = 0;
    public static final int CV_HIST_MAGIC_VAL = 1111818240;
    public static final int CV_HIST_RANGES_FLAG = 2048;
    public static final int CV_HIST_SPARSE = 1;
    public static final int CV_HIST_TREE = 1;
    public static final int CV_HIST_UNIFORM = 1;
    public static final int CV_HIST_UNIFORM_FLAG = 1024;
    public static final int CV_HeaderIsNull = -9;
    public static final int CV_KMEANS_USE_INITIAL_LABELS = 1;
    public static final int CV_L1 = 2;
    public static final int CV_L2 = 4;
    public static final double CV_LOG2 = 0.6931471805599453d;
    public static final int CV_LU = 0;
    public static final int CV_MAGIC_MASK = -65536;
    public static final int CV_MAJOR_VERSION = 4;
    public static final int CV_MATND_MAGIC_VAL = 1111687168;
    public static final int CV_MAT_CN_MASK = 4088;
    public static final int CV_MAT_CONT_FLAG = 16384;
    public static final int CV_MAT_CONT_FLAG_SHIFT = 14;
    public static final int CV_MAT_DEPTH_MASK = 7;
    public static final int CV_MAT_MAGIC_VAL = 1111621632;
    public static final int CV_MAT_TYPE_MASK = 4095;
    public static final int CV_MAX_ARR = 10;
    public static final int CV_MAX_DIM = 32;
    public static final int CV_MINMAX = 32;
    public static final int CV_MINOR_VERSION = 0;
    public static final int CV_MaskIsTiled = -26;
    public static final int CV_NORMAL = 16;
    public static final int CV_NORM_MASK = 7;
    public static final int CV_NO_CN_CHECK = 2;
    public static final int CV_NO_DEPTH_CHECK = 1;
    public static final int CV_NO_SIZE_CHECK = 4;
    public static final int CV_ORIENTED_GRAPH = 20480;
    public static final int CV_OpenCLApiCallError = -220;
    public static final int CV_OpenCLDoubleNotSupported = -221;
    public static final int CV_OpenCLInitError = -222;
    public static final int CV_OpenCLNoAMDBlasFft = -223;
    public static final int CV_OpenGlApiCallError = -219;
    public static final int CV_OpenGlNotSupported = -218;
    public static final int CV_PCA_DATA_AS_COL = 1;
    public static final int CV_PCA_DATA_AS_ROW = 0;
    public static final int CV_PCA_USE_AVG = 2;
    public static final double CV_PI = 3.141592653589793d;
    public static final int CV_QR = 4;
    public static final int CV_RAND_NORMAL = 1;
    public static final int CV_RAND_UNI = 0;
    public static final int CV_REDUCE_AVG = 1;
    public static final int CV_REDUCE_MAX = 2;
    public static final int CV_REDUCE_MIN = 3;
    public static final int CV_REDUCE_SUM = 0;
    public static final int CV_RELATIVE = 8;
    public static final int CV_RELATIVE_C = 9;
    public static final int CV_RELATIVE_L1 = 10;
    public static final int CV_RELATIVE_L2 = 12;
    public static final long CV_RNG_COEFF = 4164903690L;
    public static final int CV_SEQ_CHAIN = (CV_SEQ_ELTYPE_CODE | 4096);
    public static final int CV_SEQ_CHAIN_CONTOUR = (CV_SEQ_CHAIN | 16384);
    public static final int CV_SEQ_CONNECTED_COMP = 0;
    public static final int CV_SEQ_CONTOUR = CV_SEQ_POLYGON;
    public static final int CV_SEQ_ELTYPE_BITS = 12;
    public static final int CV_SEQ_ELTYPE_CODE = CV_8UC1;
    public static final int CV_SEQ_ELTYPE_CONNECTED_COMP = 0;
    public static final int CV_SEQ_ELTYPE_GENERIC = 0;
    public static final int CV_SEQ_ELTYPE_GRAPH_EDGE = 0;
    public static final int CV_SEQ_ELTYPE_GRAPH_VERTEX = 0;
    public static final int CV_SEQ_ELTYPE_INDEX = CV_32SC1;
    public static final int CV_SEQ_ELTYPE_MASK = 4095;
    public static final int CV_SEQ_ELTYPE_POINT = CV_32SC2;
    public static final int CV_SEQ_ELTYPE_POINT3D = CV_32FC3;
    public static final int CV_SEQ_ELTYPE_PPOINT = CV_SEQ_ELTYPE_PPOINT();
    public static final int CV_SEQ_ELTYPE_PTR = CV_SEQ_ELTYPE_PTR();
    public static final int CV_SEQ_ELTYPE_TRIAN_ATR = 0;
    public static final int CV_SEQ_FLAG_CLOSED = 16384;
    public static final int CV_SEQ_FLAG_CONVEX = 0;
    public static final int CV_SEQ_FLAG_HOLE = 32768;
    public static final int CV_SEQ_FLAG_SHIFT = 14;
    public static final int CV_SEQ_FLAG_SIMPLE = 0;
    public static final int CV_SEQ_INDEX = (0 | CV_SEQ_ELTYPE_INDEX);
    public static final int CV_SEQ_KIND_BIN_TREE = 8192;
    public static final int CV_SEQ_KIND_BITS = 2;
    public static final int CV_SEQ_KIND_CURVE = 4096;
    public static final int CV_SEQ_KIND_GENERIC = 0;
    public static final int CV_SEQ_KIND_GRAPH = 4096;
    public static final int CV_SEQ_KIND_MASK = 12288;
    public static final int CV_SEQ_KIND_SUBDIV2D = 8192;
    public static final int CV_SEQ_MAGIC_VAL = 1117323264;
    public static final int CV_SEQ_POINT3D_SET = (CV_SEQ_ELTYPE_POINT3D | 0);
    public static final int CV_SEQ_POINT_SET = (CV_SEQ_ELTYPE_POINT | 0);
    public static final int CV_SEQ_POLYGON = (CV_SEQ_POLYLINE | 16384);
    public static final int CV_SEQ_POLYGON_TREE = 8192;
    public static final int CV_SEQ_POLYLINE = (CV_SEQ_ELTYPE_POINT | 4096);
    public static final int CV_SEQ_SIMPLE_POLYGON = (CV_SEQ_POLYGON | 0);
    public static final int CV_SET_ELEM_FREE_FLAG = CV_SET_ELEM_FREE_FLAG();
    public static final int CV_SET_ELEM_IDX_MASK = 67108863;
    public static final int CV_SET_MAGIC_VAL = 1117257728;
    public static final int CV_SORT_ASCENDING = 0;
    public static final int CV_SORT_DESCENDING = 16;
    public static final int CV_SORT_EVERY_COLUMN = 1;
    public static final int CV_SORT_EVERY_ROW = 0;
    public static final int CV_SPARSE_MAT_MAGIC_VAL = 1111752704;
    public static final int CV_STATIC_ANALYSIS = 1;
    public static final int CV_STORAGE_MAGIC_VAL = 1116274688;
    public static final int CV_STRUCT_INITIALIZER = CV_STRUCT_INITIALIZER();
    public static final int CV_SUBMAT_FLAG = 32768;
    public static final int CV_SUBMAT_FLAG_SHIFT = 15;
    public static final int CV_SUBMINOR_VERSION = 1;
    public static final int CV_SVD = 1;
    public static final int CV_SVD_MODIFY_A = 1;
    public static final int CV_SVD_SYM = 2;
    public static final int CV_SVD_U_T = 2;
    public static final int CV_SVD_V_T = 4;
    public static final int CV_StsAssert = -215;
    public static final int CV_StsAutoTrace = -8;
    public static final int CV_StsBackTrace = -1;
    public static final int CV_StsBadArg = -5;
    public static final int CV_StsBadFlag = -206;
    public static final int CV_StsBadFunc = -6;
    public static final int CV_StsBadMask = -208;
    public static final int CV_StsBadMemBlock = -214;
    public static final int CV_StsBadPoint = -207;
    public static final int CV_StsBadSize = -201;
    public static final int CV_StsDivByZero = -202;
    public static final int CV_StsError = -2;
    public static final int CV_StsFilterOffsetErr = -31;
    public static final int CV_StsFilterStructContentErr = -29;
    public static final int CV_StsInplaceNotSupported = -203;
    public static final int CV_StsInternal = -3;
    public static final int CV_StsKernelStructContentErr = -30;
    public static final int CV_StsNoConv = -7;
    public static final int CV_StsNoMem = -4;
    public static final int CV_StsNotImplemented = -213;
    public static final int CV_StsNullPtr = -27;
    public static final int CV_StsObjectNotFound = -204;
    public static final int CV_StsOk = 0;
    public static final int CV_StsOutOfRange = -211;
    public static final int CV_StsParseError = -212;
    public static final int CV_StsUnmatchedFormats = -205;
    public static final int CV_StsUnmatchedSizes = -209;
    public static final int CV_StsUnsupportedFormat = -210;
    public static final int CV_StsVecLengthErr = -28;
    public static final int CV_TERMCRIT_EPS = 2;
    public static final int CV_TERMCRIT_ITER = 1;
    public static final int CV_TERMCRIT_NUMBER = 1;
    public static final String CV_TYPE_NAME_GRAPH = "opencv-graph";
    public static final String CV_TYPE_NAME_IMAGE = "opencv-image";
    public static final String CV_TYPE_NAME_MAT = "opencv-matrix";
    public static final String CV_TYPE_NAME_MATND = "opencv-nd-matrix";
    public static final String CV_TYPE_NAME_SEQ = "opencv-sequence";
    public static final String CV_TYPE_NAME_SEQ_TREE = "opencv-sequence-tree";
    public static final String CV_TYPE_NAME_SPARSE_MAT = "opencv-sparse-matrix";
    public static final String CV_VERSION = CV_VERSION();
    public static final int CV_VERSION_MAJOR = 4;
    public static final int CV_VERSION_MINOR = 0;
    public static final int CV_VERSION_REVISION = 1;
    public static final String CV_VERSION_STATUS = "";
    public static final CvSlice CV_WHOLE_ARR = cvSlice(0, CV_WHOLE_SEQ_END_INDEX);
    public static final CvSlice CV_WHOLE_SEQ = cvSlice(0, CV_WHOLE_SEQ_END_INDEX);
    public static final int CV_WHOLE_SEQ_END_INDEX = 1073741823;
    public static final int DCT_INVERSE = 1;
    public static final int DCT_ROWS = 4;
    public static final int DECOMP_CHOLESKY = 3;
    public static final int DECOMP_EIG = 2;
    public static final int DECOMP_LU = 0;
    public static final int DECOMP_NORMAL = 16;
    public static final int DECOMP_QR = 4;
    public static final int DECOMP_SVD = 1;
    public static final int DFT_COMPLEX_INPUT = 64;
    public static final int DFT_COMPLEX_OUTPUT = 16;
    public static final int DFT_INVERSE = 1;
    public static final int DFT_REAL_OUTPUT = 32;
    public static final int DFT_ROWS = 4;
    public static final int DFT_SCALE = 2;
    public static final int DYNAMIC_PARALLELISM = 35;
    public static final int FEATURE_SET_COMPUTE_10 = 10;
    public static final int FEATURE_SET_COMPUTE_11 = 11;
    public static final int FEATURE_SET_COMPUTE_12 = 12;
    public static final int FEATURE_SET_COMPUTE_13 = 13;
    public static final int FEATURE_SET_COMPUTE_20 = 20;
    public static final int FEATURE_SET_COMPUTE_21 = 21;
    public static final int FEATURE_SET_COMPUTE_30 = 30;
    public static final int FEATURE_SET_COMPUTE_32 = 32;
    public static final int FEATURE_SET_COMPUTE_35 = 35;
    public static final int FEATURE_SET_COMPUTE_50 = 50;
    public static final int FLAGS_EXPAND_SAME_NAMES = 2;
    public static final int FLAGS_MAPPING = 1;
    public static final int FLAGS_NONE = 0;
    public static final int FLOAT = 7;
    public static final int GEMM_1_T = 1;
    public static final int GEMM_2_T = 2;
    public static final int GEMM_3_T = 4;
    public static final int GLOBAL_ATOMICS = 11;
    public static final int GpuApiCallError = -217;
    public static final int GpuNotSupported = -216;
    public static final int HeaderIsNull = -9;
    public static final int IMPL_IPP = 1;
    public static final int IMPL_OPENCL = 2;
    public static final int IMPL_PLAIN = 0;
    public static final int INT = 0;
    public static final int IPL_ALIGN_16BYTES = 16;
    public static final int IPL_ALIGN_32BYTES = 32;
    public static final int IPL_ALIGN_4BYTES = 4;
    public static final int IPL_ALIGN_8BYTES = 8;
    public static final int IPL_ALIGN_DWORD = 4;
    public static final int IPL_ALIGN_QWORD = 8;
    public static final int IPL_BORDER_CONSTANT = 0;
    public static final int IPL_BORDER_REFLECT = 2;
    public static final int IPL_BORDER_REFLECT_101 = 4;
    public static final int IPL_BORDER_REPLICATE = 1;
    public static final int IPL_BORDER_TRANSPARENT = 5;
    public static final int IPL_BORDER_WRAP = 3;
    public static final int IPL_DATA_ORDER_PIXEL = 0;
    public static final int IPL_DATA_ORDER_PLANE = 1;
    public static final int IPL_DEPTH_16S = -2147483632;
    public static final int IPL_DEPTH_16U = 16;
    public static final int IPL_DEPTH_1U = 1;
    public static final int IPL_DEPTH_32F = 32;
    public static final int IPL_DEPTH_32S = -2147483616;
    public static final int IPL_DEPTH_64F = 64;
    public static final int IPL_DEPTH_8S = -2147483640;
    public static final int IPL_DEPTH_8U = 8;
    public static final int IPL_DEPTH_SIGN = Integer.MIN_VALUE;
    public static final int IPL_IMAGE_DATA = 2;
    public static final int IPL_IMAGE_HEADER = 1;
    public static final int IPL_IMAGE_MAGIC_VAL = IPL_IMAGE_MAGIC_VAL();
    public static final int IPL_IMAGE_ROI = 4;
    public static final int IPL_ORIGIN_BL = 1;
    public static final int IPL_ORIGIN_TL = 0;
    public static final int KMEANS_PP_CENTERS = 2;
    public static final int KMEANS_RANDOM_CENTERS = 0;
    public static final int KMEANS_USE_INITIAL_LABELS = 1;
    public static final int MAT = 4;
    public static final int MAT_VECTOR = 5;
    public static final int MaskIsTiled = -26;
    public static final int NATIVE_DOUBLE = 13;
    public static final int NORM_HAMMING = 6;
    public static final int NORM_HAMMING2 = 7;
    public static final int NORM_INF = 1;
    public static final int NORM_L1 = 2;
    public static final int NORM_L2 = 4;
    public static final int NORM_L2SQR = 5;
    public static final int NORM_MINMAX = 32;
    public static final int NORM_RELATIVE = 8;
    public static final int NORM_TYPE_MASK = 7;
    public static final int OCL_VECTOR_DEFAULT = 0;
    public static final int OCL_VECTOR_MAX = 1;
    public static final int OCL_VECTOR_OWN = 0;
    public static final int OPENCV_ABI_COMPATIBILITY = 400;
    public static final int OpenCLApiCallError = -220;
    public static final int OpenCLDoubleNotSupported = -221;
    public static final int OpenCLInitError = -222;
    public static final int OpenCLNoAMDBlasFft = -223;
    public static final int OpenGlApiCallError = -219;
    public static final int OpenGlNotSupported = -218;
    public static final int REAL = 2;
    public static final int REDUCE_AVG = 1;
    public static final int REDUCE_MAX = 2;
    public static final int REDUCE_MIN = 3;
    public static final int REDUCE_SUM = 0;
    public static final int ROTATE_180 = 1;
    public static final int ROTATE_90_CLOCKWISE = 0;
    public static final int ROTATE_90_COUNTERCLOCKWISE = 2;
    public static final int SCALAR = 12;
    public static final int SHARED_ATOMICS = 12;
    public static final int SOLVELP_MULTI = 1;
    public static final int SOLVELP_SINGLE = 0;
    public static final int SOLVELP_UNBOUNDED = -2;
    public static final int SOLVELP_UNFEASIBLE = -1;
    public static final int SORT_ASCENDING = 0;
    public static final int SORT_DESCENDING = 16;
    public static final int SORT_EVERY_COLUMN = 1;
    public static final int SORT_EVERY_ROW = 0;
    public static final int STRING = 3;
    public static final int StsAssert = -215;
    public static final int StsAutoTrace = -8;
    public static final int StsBackTrace = -1;
    public static final int StsBadArg = -5;
    public static final int StsBadFlag = -206;
    public static final int StsBadFunc = -6;
    public static final int StsBadMask = -208;
    public static final int StsBadMemBlock = -214;
    public static final int StsBadPoint = -207;
    public static final int StsBadSize = -201;
    public static final int StsDivByZero = -202;
    public static final int StsError = -2;
    public static final int StsFilterOffsetErr = -31;
    public static final int StsFilterStructContentErr = -29;
    public static final int StsInplaceNotSupported = -203;
    public static final int StsInternal = -3;
    public static final int StsKernelStructContentErr = -30;
    public static final int StsNoConv = -7;
    public static final int StsNoMem = -4;
    public static final int StsNotImplemented = -213;
    public static final int StsNullPtr = -27;
    public static final int StsObjectNotFound = -204;
    public static final int StsOk = 0;
    public static final int StsOutOfRange = -211;
    public static final int StsParseError = -212;
    public static final int StsUnmatchedFormats = -205;
    public static final int StsUnmatchedSizes = -209;
    public static final int StsUnsupportedFormat = -210;
    public static final int StsVecLengthErr = -28;
    public static final int TYPE_FUN = 3;
    public static final int TYPE_GENERAL = 0;
    public static final int TYPE_MARKER = 1;
    public static final int TYPE_WRAPPER = 2;
    public static final int UCHAR = 11;
    public static final int UINT64 = 9;
    public static final int UNSIGNED_INT = 8;
    public static final int USAGE_ALLOCATE_DEVICE_MEMORY = 2;
    public static final int USAGE_ALLOCATE_HOST_MEMORY = 1;
    public static final int USAGE_ALLOCATE_SHARED_MEMORY = 4;
    public static final int USAGE_DEFAULT = 0;
    public static final int WARP_SHUFFLE_FUNCTIONS = 30;
    public static final int __UMAT_USAGE_FLAGS_32BIT = Integer.MAX_VALUE;
    public static final String cvFuncName = "";

    @MemberGetter
    public static native int CV_16FC1();

    @MemberGetter
    public static native int CV_16FC2();

    @MemberGetter
    public static native int CV_16FC3();

    @MemberGetter
    public static native int CV_16FC4();

    public static native int CV_16SC(int i);

    public static native int CV_16UC(int i);

    public static native int CV_32FC(int i);

    public static native int CV_32SC(int i);

    public static native int CV_64FC(int i);

    public static native int CV_8SC(int i);

    public static native int CV_8UC(int i);

    @MemberGetter
    public static native int CV_FP16_TYPE();

    public static native int CV_IS_CONT_MAT(int i);

    public static native int CV_IS_MAT_CONT(int i);

    public static native int CV_MAKETYPE(int i, int i2);

    public static native int CV_MAKE_TYPE(int i, int i2);

    public static native int CV_MAT_CN(int i);

    public static native int CV_MAT_DEPTH(int i);

    public static native int CV_MAT_TYPE(int i);

    @MemberGetter
    public static native int CV_SEQ_ELTYPE_PPOINT();

    @MemberGetter
    public static native int CV_SEQ_ELTYPE_PTR();

    @MemberGetter
    public static native int CV_SET_ELEM_FREE_FLAG();

    @MemberGetter
    public static native int CV_STRUCT_INITIALIZER();

    @MemberGetter
    public static native String CV_VERSION();

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky(DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, int i, DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky(FloatBuffer floatBuffer, @Cast({"size_t"}) long j, int i, FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky(DoublePointer doublePointer, @Cast({"size_t"}) long j, int i, DoublePointer doublePointer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky(FloatPointer floatPointer, @Cast({"size_t"}) long j, int i, FloatPointer floatPointer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky(double[] dArr, @Cast({"size_t"}) long j, int i, double[] dArr2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky(float[] fArr, @Cast({"size_t"}) long j, int i, float[] fArr2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky32f(FloatBuffer floatBuffer, @Cast({"size_t"}) long j, int i, FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky32f(FloatPointer floatPointer, @Cast({"size_t"}) long j, int i, FloatPointer floatPointer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky32f(float[] fArr, @Cast({"size_t"}) long j, int i, float[] fArr2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky64f(DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, int i, DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky64f(DoublePointer doublePointer, @Cast({"size_t"}) long j, int i, DoublePointer doublePointer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    @Cast({"bool"})
    public static native boolean Cholesky64f(double[] dArr, @Cast({"size_t"}) long j, int i, double[] dArr2, @Cast({"size_t"}) long j2, int i2);

    @MemberGetter
    public static native int IPL_IMAGE_MAGIC_VAL();

    @Namespace("cv::hal")
    public static native int LU(DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, int i, DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU(FloatBuffer floatBuffer, @Cast({"size_t"}) long j, int i, FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU(DoublePointer doublePointer, @Cast({"size_t"}) long j, int i, DoublePointer doublePointer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU(FloatPointer floatPointer, @Cast({"size_t"}) long j, int i, FloatPointer floatPointer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU(double[] dArr, @Cast({"size_t"}) long j, int i, double[] dArr2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU(float[] fArr, @Cast({"size_t"}) long j, int i, float[] fArr2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU32f(FloatBuffer floatBuffer, @Cast({"size_t"}) long j, int i, FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU32f(FloatPointer floatPointer, @Cast({"size_t"}) long j, int i, FloatPointer floatPointer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU32f(float[] fArr, @Cast({"size_t"}) long j, int i, float[] fArr2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU64f(DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, int i, DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU64f(DoublePointer doublePointer, @Cast({"size_t"}) long j, int i, DoublePointer doublePointer2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv::hal")
    public static native int LU64f(double[] dArr, @Cast({"size_t"}) long j, int i, double[] dArr2, @Cast({"size_t"}) long j2, int i2);

    @Namespace("cv")
    public static native void LUT(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void LUT(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void LUT(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native double Mahalanobis(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native double Mahalanobis(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native double Mahalanobis(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void PCABackProject(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4);

    @Namespace("cv")
    public static native void PCABackProject(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4);

    @Namespace("cv")
    public static native void PCABackProject(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4);

    @Namespace("cv")
    public static native void PCACompute(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void PCACompute(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, double d);

    @Namespace("cv")
    public static native void PCACompute(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, int i);

    @Namespace("cv")
    public static native void PCACompute(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void PCACompute(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, double d);

    @Namespace("cv")
    public static native void PCACompute(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, int i);

    @Namespace("cv")
    public static native void PCACompute(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void PCACompute(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, double d);

    @Namespace("cv")
    public static native void PCACompute(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, int i);

    @Namespace("cv")
    @Name({"PCACompute"})
    public static native void PCACompute2(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4);

    @Namespace("cv")
    @Name({"PCACompute"})
    public static native void PCACompute2(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4, double d);

    @Namespace("cv")
    @Name({"PCACompute"})
    public static native void PCACompute2(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4, int i);

    @Namespace("cv")
    @Name({"PCACompute"})
    public static native void PCACompute2(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4);

    @Namespace("cv")
    @Name({"PCACompute"})
    public static native void PCACompute2(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4, double d);

    @Namespace("cv")
    @Name({"PCACompute"})
    public static native void PCACompute2(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4, int i);

    @Namespace("cv")
    @Name({"PCACompute"})
    public static native void PCACompute2(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4);

    @Namespace("cv")
    @Name({"PCACompute"})
    public static native void PCACompute2(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4, double d);

    @Namespace("cv")
    @Name({"PCACompute"})
    public static native void PCACompute2(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4, int i);

    @Namespace("cv")
    public static native void PCAProject(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4);

    @Namespace("cv")
    public static native void PCAProject(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4);

    @Namespace("cv")
    public static native void PCAProject(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4);

    @Namespace("cv")
    public static native double PSNR(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native double PSNR(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, double d);

    @Namespace("cv")
    public static native double PSNR(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native double PSNR(@ByVal Mat mat, @ByVal Mat mat2, double d);

    @Namespace("cv")
    public static native double PSNR(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native double PSNR(@ByVal UMat uMat, @ByVal UMat uMat2, double d);

    @Namespace("cv::hal")
    public static native int QR32f(FloatBuffer floatBuffer, @Cast({"size_t"}) long j, int i, int i2, int i3, FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3);

    @Namespace("cv::hal")
    public static native int QR32f(FloatPointer floatPointer, @Cast({"size_t"}) long j, int i, int i2, int i3, FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3);

    @Namespace("cv::hal")
    public static native int QR32f(float[] fArr, @Cast({"size_t"}) long j, int i, int i2, int i3, float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3);

    @Namespace("cv::hal")
    public static native int QR64f(DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, int i, int i2, int i3, DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3);

    @Namespace("cv::hal")
    public static native int QR64f(DoublePointer doublePointer, @Cast({"size_t"}) long j, int i, int i2, int i3, DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3);

    @Namespace("cv::hal")
    public static native int QR64f(double[] dArr, @Cast({"size_t"}) long j, int i, int i2, int i3, double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3);

    @Namespace("cv")
    public static native void SVBackSubst(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4, @ByVal GpuMat gpuMat5);

    @Namespace("cv")
    public static native void SVBackSubst(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4, @ByVal Mat mat5);

    @Namespace("cv")
    public static native void SVBackSubst(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4, @ByVal UMat uMat5);

    @Namespace("cv::hal")
    public static native void SVD32f(FloatBuffer floatBuffer, @Cast({"size_t"}) long j, FloatBuffer floatBuffer2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer4, @Cast({"size_t"}) long j3, int i, int i2, int i3);

    @Namespace("cv::hal")
    public static native void SVD32f(FloatPointer floatPointer, @Cast({"size_t"}) long j, FloatPointer floatPointer2, FloatPointer floatPointer3, @Cast({"size_t"}) long j2, FloatPointer floatPointer4, @Cast({"size_t"}) long j3, int i, int i2, int i3);

    @Namespace("cv::hal")
    public static native void SVD32f(float[] fArr, @Cast({"size_t"}) long j, float[] fArr2, float[] fArr3, @Cast({"size_t"}) long j2, float[] fArr4, @Cast({"size_t"}) long j3, int i, int i2, int i3);

    @Namespace("cv::hal")
    public static native void SVD64f(DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, DoubleBuffer doubleBuffer2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer4, @Cast({"size_t"}) long j3, int i, int i2, int i3);

    @Namespace("cv::hal")
    public static native void SVD64f(DoublePointer doublePointer, @Cast({"size_t"}) long j, DoublePointer doublePointer2, DoublePointer doublePointer3, @Cast({"size_t"}) long j2, DoublePointer doublePointer4, @Cast({"size_t"}) long j3, int i, int i2, int i3);

    @Namespace("cv::hal")
    public static native void SVD64f(double[] dArr, @Cast({"size_t"}) long j, double[] dArr2, double[] dArr3, @Cast({"size_t"}) long j2, double[] dArr4, @Cast({"size_t"}) long j3, int i, int i2, int i3);

    @Namespace("cv")
    public static native void SVDecomp(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4);

    @Namespace("cv")
    public static native void SVDecomp(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4, int i);

    @Namespace("cv")
    public static native void SVDecomp(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4);

    @Namespace("cv")
    public static native void SVDecomp(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4, int i);

    @Namespace("cv")
    public static native void SVDecomp(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4);

    @Namespace("cv")
    public static native void SVDecomp(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4, int i);

    @Namespace("cv")
    @Cast({"uchar"})
    public static native byte abs(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @Cast({"unsigned"})
    public static native int abs(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @ByVal
    public static native MatExpr abs(@ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    public static native MatExpr abs(@ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @Cast({"ushort"})
    public static native short abs(@Cast({"ushort"}) short s);

    @Namespace("cv")
    public static native void absdiff(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void absdiff(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void absdiff(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv::hal")
    public static native void absdiff16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, IntBuffer intBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, IntPointer intPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, int[] iArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void absdiff8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native MatExpr add(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native MatExpr add(@ByRef @Const Mat mat, @ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native MatExpr add(@ByRef @Const Mat mat, @ByRef @Const Scalar scalar);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native MatExpr add(@ByRef @Const MatExpr matExpr, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native MatExpr add(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native MatExpr add(@ByRef @Const MatExpr matExpr, @ByRef @Const Scalar scalar);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native MatExpr add(@ByRef @Const Scalar scalar, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native MatExpr add(@ByRef @Const Scalar scalar, @ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native Range add(int i, @ByRef @Const Range range);

    @Namespace("cv")
    @ByVal
    @Name({"operator +"})
    public static native Range add(@ByRef @Const Range range, int i);

    @Namespace("cv")
    public static native void add(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void add(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4, int i);

    @Namespace("cv")
    public static native void add(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void add(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4, int i);

    @Namespace("cv")
    public static native void add(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void add(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4, int i);

    @Namespace("cv::hal")
    public static native void add16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, IntBuffer intBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, IntPointer intPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, int[] iArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void add8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    @ByRef
    @Name({"operator +="})
    public static native Mat addPut(@ByRef Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByRef
    @Name({"operator +="})
    public static native Mat addPut(@ByRef Mat mat, @ByRef @Const Scalar scalar);

    @Namespace("cv::hal")
    public static native void addRNGBias32f(FloatBuffer floatBuffer, @Const FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native void addRNGBias32f(FloatPointer floatPointer, @Const FloatPointer floatPointer2, int i);

    @Namespace("cv::hal")
    public static native void addRNGBias32f(float[] fArr, @Const float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native void addRNGBias64f(DoubleBuffer doubleBuffer, @Const DoubleBuffer doubleBuffer2, int i);

    @Namespace("cv::hal")
    public static native void addRNGBias64f(DoublePointer doublePointer, @Const DoublePointer doublePointer2, int i);

    @Namespace("cv::hal")
    public static native void addRNGBias64f(double[] dArr, @Const double[] dArr2, int i);

    @Namespace("cv::samples")
    public static native void addSamplesDataSearchPath(@opencv_core.Str String str);

    @Namespace("cv::samples")
    public static native void addSamplesDataSearchPath(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::samples")
    public static native void addSamplesDataSearchSubDirectory(@opencv_core.Str String str);

    @Namespace("cv::samples")
    public static native void addSamplesDataSearchSubDirectory(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv")
    public static native void addWeighted(@ByVal GpuMat gpuMat, double d, @ByVal GpuMat gpuMat2, double d2, double d3, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void addWeighted(@ByVal GpuMat gpuMat, double d, @ByVal GpuMat gpuMat2, double d2, double d3, @ByVal GpuMat gpuMat3, int i);

    @Namespace("cv")
    public static native void addWeighted(@ByVal Mat mat, double d, @ByVal Mat mat2, double d2, double d3, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void addWeighted(@ByVal Mat mat, double d, @ByVal Mat mat2, double d2, double d3, @ByVal Mat mat3, int i);

    @Namespace("cv")
    public static native void addWeighted(@ByVal UMat uMat, double d, @ByVal UMat uMat2, double d2, double d3, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void addWeighted(@ByVal UMat uMat, double d, @ByVal UMat uMat2, double d2, double d3, @ByVal UMat uMat3, int i);

    @Namespace("cv::hal")
    public static native void addWeighted16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, IntBuffer intBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, IntPointer intPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, int[] iArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void addWeighted8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    @Cast({"size_t"})
    public static native long alignSize(@Cast({"size_t"}) long j, int i);

    @Namespace("cv")
    @Cast({"cv::AccessFlag"})
    @Name({"operator &"})
    public static native int and(@Cast({"const cv::AccessFlag"}) int i, @Cast({"const cv::AccessFlag"}) int i2);

    @Namespace("cv")
    @ByVal
    @Name({"operator &"})
    public static native MatExpr and(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByVal
    @Name({"operator &"})
    public static native MatExpr and(@ByRef @Const Mat mat, @ByRef @Const Scalar scalar);

    @Namespace("cv")
    @ByVal
    @Name({"operator &"})
    public static native MatExpr and(@ByRef @Const Scalar scalar, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator &"})
    public static native Range and(@ByRef @Const Range range, @ByRef @Const Range range2);

    @Namespace("cv::hal")
    public static native void and8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void and8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void and8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @ByRef
    @Cast({"cv::AccessFlag*"})
    @Name({"operator &="})
    @Namespace("cv")
    public static native IntBuffer andPut(@ByRef @Cast({"cv::AccessFlag*"}) IntBuffer intBuffer, @Cast({"const cv::AccessFlag"}) int i);

    @ByRef
    @Cast({"cv::AccessFlag*"})
    @Name({"operator &="})
    @Namespace("cv")
    public static native IntPointer andPut(@ByRef @Cast({"cv::AccessFlag*"}) IntPointer intPointer, @Cast({"const cv::AccessFlag"}) int i);

    @Namespace("cv")
    @ByRef
    @Name({"operator &="})
    public static native Mat andPut(@ByRef Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByRef
    @Name({"operator &="})
    public static native Mat andPut(@ByRef Mat mat, @ByRef @Const Scalar scalar);

    @Namespace("cv")
    @ByRef
    @Name({"operator &="})
    public static native Range andPut(@ByRef Range range, @ByRef @Const Range range2);

    @ByRef
    @Cast({"cv::AccessFlag*"})
    @Name({"operator &="})
    @Namespace("cv")
    public static native int[] andPut(@ByRef @Cast({"cv::AccessFlag*"}) int[] iArr, @Cast({"const cv::AccessFlag"}) int i);

    @Namespace("cv::ocl")
    public static native void attachContext(@opencv_core.Str String str, Pointer pointer, Pointer pointer2, Pointer pointer3);

    @Namespace("cv::ocl")
    public static native void attachContext(@opencv_core.Str BytePointer bytePointer, Pointer pointer, Pointer pointer2, Pointer pointer3);

    @Namespace("cv")
    public static native void batchDistance(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, int i, @ByVal GpuMat gpuMat4);

    @Namespace("cv")
    public static native void batchDistance(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, int i, @ByVal GpuMat gpuMat4, int i2, int i3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat5, int i4, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void batchDistance(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, int i, @ByVal Mat mat4);

    @Namespace("cv")
    public static native void batchDistance(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, int i, @ByVal Mat mat4, int i2, int i3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat5, int i4, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void batchDistance(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, int i, @ByVal UMat uMat4);

    @Namespace("cv")
    public static native void batchDistance(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, int i, @ByVal UMat uMat4, int i2, int i3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat5, int i4, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void bitwise_and(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void bitwise_and(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4);

    @Namespace("cv")
    public static native void bitwise_and(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void bitwise_and(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4);

    @Namespace("cv")
    public static native void bitwise_and(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void bitwise_and(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4);

    @Namespace("cv")
    public static native void bitwise_not(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void bitwise_not(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat3);

    @Namespace("cv")
    public static native void bitwise_not(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void bitwise_not(@ByVal Mat mat, @ByVal Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat3);

    @Namespace("cv")
    public static native void bitwise_not(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native void bitwise_not(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat3);

    @Namespace("cv")
    public static native void bitwise_or(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void bitwise_or(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4);

    @Namespace("cv")
    public static native void bitwise_or(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void bitwise_or(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4);

    @Namespace("cv")
    public static native void bitwise_or(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void bitwise_or(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4);

    @Namespace("cv")
    public static native void bitwise_xor(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void bitwise_xor(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4);

    @Namespace("cv")
    public static native void bitwise_xor(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void bitwise_xor(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4);

    @Namespace("cv")
    public static native void bitwise_xor(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void bitwise_xor(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4);

    @Namespace("cv")
    public static native int borderInterpolate(int i, int i2, int i3);

    @Namespace("cv::ocl")
    public static native void buildOptionsAddMatrixDescription(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal GpuMat gpuMat);

    @Namespace("cv::ocl")
    public static native void buildOptionsAddMatrixDescription(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal Mat mat);

    @Namespace("cv::ocl")
    public static native void buildOptionsAddMatrixDescription(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal UMat uMat);

    @Namespace("cv::ocl")
    public static native void buildOptionsAddMatrixDescription(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal GpuMat gpuMat);

    @Namespace("cv::ocl")
    public static native void buildOptionsAddMatrixDescription(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal Mat mat);

    @Namespace("cv::ocl")
    public static native void buildOptionsAddMatrixDescription(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void calcCovarMatrix(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, int i);

    @Namespace("cv")
    public static native void calcCovarMatrix(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, int i, int i2);

    @Namespace("cv")
    public static native void calcCovarMatrix(@Const Mat mat, int i, @ByRef Mat mat2, @ByRef Mat mat3, int i2);

    @Namespace("cv")
    public static native void calcCovarMatrix(@Const Mat mat, int i, @ByRef Mat mat2, @ByRef Mat mat3, int i2, int i3);

    @Namespace("cv")
    public static native void calcCovarMatrix(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, int i);

    @Namespace("cv")
    public static native void calcCovarMatrix(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, int i, int i2);

    @Namespace("cv")
    public static native void calcCovarMatrix(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, int i);

    @Namespace("cv")
    public static native void calcCovarMatrix(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, int i, int i2);

    @Namespace("cv")
    public static native void cartToPolar(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4);

    @Namespace("cv")
    public static native void cartToPolar(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void cartToPolar(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4);

    @Namespace("cv")
    public static native void cartToPolar(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void cartToPolar(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4);

    @Namespace("cv")
    public static native void cartToPolar(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4, @Cast({"bool"}) boolean z);

    @Namespace("cv::details")
    @Cast({"char"})
    public static native byte char_tolower(@Cast({"char"}) byte b);

    @Namespace("cv::details")
    @Cast({"char"})
    public static native byte char_toupper(@Cast({"char"}) byte b);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkHardwareSupport(int i);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntBuffer intBuffer, @ByVal GpuMat gpuMat);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntBuffer intBuffer, @ByVal GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntBuffer intBuffer, @ByVal Mat mat);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntBuffer intBuffer, @ByVal Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntBuffer intBuffer, @ByVal UMat uMat);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntBuffer intBuffer, @ByVal UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntPointer intPointer, @ByVal GpuMat gpuMat);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntPointer intPointer, @ByVal GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntPointer intPointer, @ByVal Mat mat);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntPointer intPointer, @ByVal Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntPointer intPointer, @ByVal UMat uMat);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const IntPointer intPointer, @ByVal UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const int[] iArr, @ByVal GpuMat gpuMat);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const int[] iArr, @ByVal GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const int[] iArr, @ByVal Mat mat);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const int[] iArr, @ByVal Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const int[] iArr, @ByVal UMat uMat);

    @Namespace("cv::ocl")
    public static native int checkOptimalVectorWidth(@Const int[] iArr, @ByVal UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkRange(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkRange(@ByVal GpuMat gpuMat, @Cast({"bool"}) boolean z, Point point, double d, double d2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkRange(@ByVal Mat mat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkRange(@ByVal Mat mat, @Cast({"bool"}) boolean z, Point point, double d, double d2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkRange(@ByVal UMat uMat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkRange(@ByVal UMat uMat, @Cast({"bool"}) boolean z, Point point, double d, double d2);

    @Namespace("cv")
    public static native void clearSeq(CvSeq cvSeq);

    @Namespace("cv::hal")
    public static native void cmp16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void cmp8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    public static native void compare(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, int i);

    @Namespace("cv")
    public static native void compare(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, int i);

    @Namespace("cv")
    public static native void compare(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, int i);

    @Namespace("cv")
    public static native void completeSymm(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void completeSymm(@ByVal GpuMat gpuMat, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void completeSymm(@ByVal Mat mat);

    @Namespace("cv")
    public static native void completeSymm(@ByVal Mat mat, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void completeSymm(@ByVal UMat uMat);

    @Namespace("cv")
    public static native void completeSymm(@ByVal UMat uMat, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void convertFp16(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void convertFp16(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void convertFp16(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv::ocl")
    public static native void convertFromBuffer(Pointer pointer, @Cast({"size_t"}) long j, int i, int i2, int i3, @ByRef UMat uMat);

    @Namespace("cv::ocl")
    public static native void convertFromImage(Pointer pointer, @ByRef UMat uMat);

    @Namespace("cv")
    public static native void convertScaleAbs(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void convertScaleAbs(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, double d, double d2);

    @Namespace("cv")
    public static native void convertScaleAbs(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void convertScaleAbs(@ByVal Mat mat, @ByVal Mat mat2, double d, double d2);

    @Namespace("cv")
    public static native void convertScaleAbs(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native void convertScaleAbs(@ByVal UMat uMat, @ByVal UMat uMat2, double d, double d2);

    @Namespace("cv::ocl")
    public static native String convertTypeStr(int i, int i2, int i3, @Cast({"char*"}) ByteBuffer byteBuffer);

    @Namespace("cv::ocl")
    @Cast({"const char*"})
    public static native BytePointer convertTypeStr(int i, int i2, int i3, @Cast({"char*"}) BytePointer bytePointer);

    @Namespace("cv::ocl")
    @Cast({"const char*"})
    public static native BytePointer convertTypeStr(int i, int i2, int i3, @Cast({"char*"}) byte[] bArr);

    @Namespace("cv")
    public static native void copyMakeBorder(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, int i2, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void copyMakeBorder(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, int i2, int i3, int i4, int i5, @ByRef(nullValue = "cv::Scalar()") @Const Scalar scalar);

    @Namespace("cv")
    public static native void copyMakeBorder(@ByVal Mat mat, @ByVal Mat mat2, int i, int i2, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void copyMakeBorder(@ByVal Mat mat, @ByVal Mat mat2, int i, int i2, int i3, int i4, int i5, @ByRef(nullValue = "cv::Scalar()") @Const Scalar scalar);

    @Namespace("cv")
    public static native void copyMakeBorder(@ByVal UMat uMat, @ByVal UMat uMat2, int i, int i2, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void copyMakeBorder(@ByVal UMat uMat, @ByVal UMat uMat2, int i, int i2, int i3, int i4, int i5, @ByRef(nullValue = "cv::Scalar()") @Const Scalar scalar);

    @Namespace("cv")
    public static native void copyTo(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void copyTo(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void copyTo(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native int countNonZero(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native int countNonZero(@ByVal Mat mat);

    @Namespace("cv")
    public static native int countNonZero(@ByVal UMat uMat);

    @Namespace("cv::cuda")
    public static native void createContinuous(int i, int i2, int i3, @ByVal GpuMat gpuMat);

    @Namespace("cv::cuda")
    public static native void createContinuous(int i, int i2, int i3, @ByVal Mat mat);

    @Namespace("cv::cuda")
    public static native void createContinuous(int i, int i2, int i3, @ByVal UMat uMat);

    @Namespace("cv")
    public static native float cubeRoot(float f);

    public static native void cvAbsDiff(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvAbsDiffS(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, @ByVal CvScalar cvScalar);

    public static native void cvAdd(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvAdd(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, @Const opencv_core.CvArr cvArr4);

    public static native void cvAddS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2);

    public static native void cvAddS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3);

    public static native void cvAddWeighted(@Const opencv_core.CvArr cvArr, double d, @Const opencv_core.CvArr cvArr2, double d2, double d3, opencv_core.CvArr cvArr3);

    public static native Pointer cvAlloc(@Cast({"size_t"}) long j);

    public static native void cvAnd(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvAnd(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, @Const opencv_core.CvArr cvArr4);

    public static native void cvAndS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2);

    public static native void cvAndS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3);

    @ByVal
    public static native CvScalar cvAvg(@Const opencv_core.CvArr cvArr);

    @ByVal
    public static native CvScalar cvAvg(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2);

    public static native void cvAvgSdv(@Const opencv_core.CvArr cvArr, CvScalar cvScalar, CvScalar cvScalar2);

    public static native void cvAvgSdv(@Const opencv_core.CvArr cvArr, CvScalar cvScalar, CvScalar cvScalar2, @Const opencv_core.CvArr cvArr2);

    public static native void cvBackProjectPCA(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4);

    @ByVal
    public static native CvBox2D cvBox2D();

    @ByVal
    public static native CvBox2D cvBox2D(@Cast({"CvPoint2D32f*"}) @ByVal(nullValue = "CvPoint2D32f()") FloatBuffer floatBuffer, @ByVal(nullValue = "CvSize2D32f()") CvSize2D32f cvSize2D32f, float f);

    @ByVal
    public static native CvBox2D cvBox2D(@ByVal(nullValue = "CvPoint2D32f()") CvPoint2D32f cvPoint2D32f, @ByVal(nullValue = "CvSize2D32f()") CvSize2D32f cvSize2D32f, float f);

    @ByVal
    public static native CvBox2D cvBox2D(@ByRef @Const RotatedRect rotatedRect);

    @ByVal
    public static native CvBox2D cvBox2D(@Cast({"CvPoint2D32f*"}) @ByVal(nullValue = "CvPoint2D32f()") float[] fArr, @ByVal(nullValue = "CvSize2D32f()") CvSize2D32f cvSize2D32f, float f);

    public static native void cvCalcCovarMatrix(@Cast({"const CvArr**"}) PointerPointer pointerPointer, int i, opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i2);

    public static native void cvCalcCovarMatrix(@ByPtrPtr @Const opencv_core.CvArr cvArr, int i, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, int i2);

    public static native void cvCalcPCA(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4, int i);

    public static native void cvCartToPolar(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvCartToPolar(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4, int i);

    public static native float cvCbrt(float f);

    public static native int cvCeil(double d);

    public static native int cvCeil(float f);

    public static native int cvCeil(int i);

    public static native void cvChangeSeqBlock(Pointer pointer, int i);

    public static native int cvCheckArr(@Const opencv_core.CvArr cvArr);

    public static native int cvCheckArr(@Const opencv_core.CvArr cvArr, int i, double d, double d2);

    public static native int cvCheckArray(opencv_core.CvArr cvArr, int i, double d, double d2);

    public static native int cvCheckHardwareSupport(int i);

    @ByVal
    public static native CvTermCriteria cvCheckTermCriteria(@ByVal CvTermCriteria cvTermCriteria, double d, int i);

    public static native void cvClearGraph(CvGraph cvGraph);

    public static native void cvClearMemStorage(CvMemStorage cvMemStorage);

    public static native void cvClearND(opencv_core.CvArr cvArr, @Const IntBuffer intBuffer);

    public static native void cvClearND(opencv_core.CvArr cvArr, @Const IntPointer intPointer);

    public static native void cvClearND(opencv_core.CvArr cvArr, @Const int[] iArr);

    public static native void cvClearSeq(CvSeq cvSeq);

    public static native void cvClearSet(CvSet cvSet);

    public static native Pointer cvClone(@Const Pointer pointer);

    public static native CvGraph cvCloneGraph(@Const CvGraph cvGraph, CvMemStorage cvMemStorage);

    public static native IplImage cvCloneImage(@Const IplImage iplImage);

    public static native CvMat cvCloneMat(@Const CvMat cvMat);

    public static native CvMatND cvCloneMatND(@Const CvMatND cvMatND);

    public static native CvSeq cvCloneSeq(@Const CvSeq cvSeq);

    public static native CvSeq cvCloneSeq(@Const CvSeq cvSeq, CvMemStorage cvMemStorage);

    public static native CvSparseMat cvCloneSparseMat(@Const CvSparseMat cvSparseMat);

    public static native void cvCmp(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, int i);

    public static native void cvCmpS(@Const opencv_core.CvArr cvArr, double d, opencv_core.CvArr cvArr2, int i);

    public static native void cvCompleteSymm(CvMat cvMat);

    public static native void cvCompleteSymm(CvMat cvMat, int i);

    public static native void cvConvert(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native void cvConvertScale(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native void cvConvertScale(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, double d, double d2);

    public static native void cvConvertScaleAbs(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native void cvConvertScaleAbs(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, double d, double d2);

    public static native void cvCopy(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native void cvCopy(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3);

    public static native int cvCountNonZero(@Const opencv_core.CvArr cvArr);

    public static native CvMemStorage cvCreateChildMemStorage(CvMemStorage cvMemStorage);

    public static native void cvCreateData(opencv_core.CvArr cvArr);

    public static native CvGraph cvCreateGraph(int i, int i2, int i3, int i4, CvMemStorage cvMemStorage);

    public static native CvGraphScanner cvCreateGraphScanner(CvGraph cvGraph);

    public static native CvGraphScanner cvCreateGraphScanner(CvGraph cvGraph, CvGraphVtx cvGraphVtx, int i);

    public static native IplImage cvCreateImage(@ByVal CvSize cvSize, int i, int i2);

    public static native IplImage cvCreateImageHeader(@ByVal CvSize cvSize, int i, int i2);

    public static native CvMat cvCreateMat(int i, int i2, int i3);

    public static native CvMat cvCreateMatHeader(int i, int i2, int i3);

    public static native CvMatND cvCreateMatND(int i, @Const IntBuffer intBuffer, int i2);

    public static native CvMatND cvCreateMatND(int i, @Const IntPointer intPointer, int i2);

    public static native CvMatND cvCreateMatND(int i, @Const int[] iArr, int i2);

    public static native CvMatND cvCreateMatNDHeader(int i, @Const IntBuffer intBuffer, int i2);

    public static native CvMatND cvCreateMatNDHeader(int i, @Const IntPointer intPointer, int i2);

    public static native CvMatND cvCreateMatNDHeader(int i, @Const int[] iArr, int i2);

    public static native CvMemStorage cvCreateMemStorage();

    public static native CvMemStorage cvCreateMemStorage(int i);

    public static native CvSeq cvCreateSeq(int i, @Cast({"size_t"}) long j, @Cast({"size_t"}) long j2, CvMemStorage cvMemStorage);

    public static native void cvCreateSeqBlock(CvSeqWriter cvSeqWriter);

    public static native CvSet cvCreateSet(int i, int i2, int i3, CvMemStorage cvMemStorage);

    public static native CvSparseMat cvCreateSparseMat(int i, @Const IntBuffer intBuffer, int i2);

    public static native CvSparseMat cvCreateSparseMat(int i, @Const IntPointer intPointer, int i2);

    public static native CvSparseMat cvCreateSparseMat(int i, @Const int[] iArr, int i2);

    public static native CvSparseMat cvCreateSparseMat(@ByRef @Const SparseMat sparseMat);

    public static native void cvCrossProduct(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvCvtScale(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, double d, double d2);

    public static native void cvCvtScaleAbs(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, double d, double d2);

    public static native Pointer cvCvtSeqToArray(@Const CvSeq cvSeq, Pointer pointer);

    public static native Pointer cvCvtSeqToArray(@Const CvSeq cvSeq, Pointer pointer, @ByVal(nullValue = "CvSlice(CV_WHOLE_SEQ)") CvSlice cvSlice);

    public static native void cvDCT(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i);

    public static native void cvDFT(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i);

    public static native void cvDFT(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i, int i2);

    public static native void cvDecRefData(opencv_core.CvArr cvArr);

    public static native double cvDet(@Const opencv_core.CvArr cvArr);

    public static native void cvDiv(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvDiv(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, double d);

    public static native double cvDotProduct(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2);

    public static native void cvEigenVV(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvEigenVV(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, double d, int i, int i2);

    public static native CvSeq cvEndWriteSeq(CvSeqWriter cvSeqWriter);

    public static native void cvError(int i, String str, String str2, String str3, int i2);

    public static native void cvError(int i, @Cast({"const char*"}) BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"const char*"}) BytePointer bytePointer3, int i2);

    public static native int cvErrorFromIppStatus(int i);

    @Cast({"const char*"})
    public static native BytePointer cvErrorStr(int i);

    public static native void cvExp(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native void cvFFT(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i, int i2);

    public static native float cvFastArctan(float f, float f2);

    public static native CvGraphEdge cvFindGraphEdge(@Const CvGraph cvGraph, int i, int i2);

    public static native CvGraphEdge cvFindGraphEdgeByPtr(@Const CvGraph cvGraph, @Const CvGraphVtx cvGraphVtx, @Const CvGraphVtx cvGraphVtx2);

    public static native void cvFlip(@Const opencv_core.CvArr cvArr);

    public static native void cvFlip(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i);

    public static native int cvFloor(double d);

    public static native int cvFloor(float f);

    public static native int cvFloor(int i);

    public static native void cvFlushSeqWriter(CvSeqWriter cvSeqWriter);

    public static native void cvFree_(Pointer pointer);

    public static native void cvGEMM(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, double d, @Const opencv_core.CvArr cvArr3, double d2, opencv_core.CvArr cvArr4);

    public static native void cvGEMM(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, double d, @Const opencv_core.CvArr cvArr3, double d2, opencv_core.CvArr cvArr4, int i);

    @ByVal
    public static native CvScalar cvGet1D(@Const opencv_core.CvArr cvArr, int i);

    @ByVal
    public static native CvScalar cvGet2D(@Const opencv_core.CvArr cvArr, int i, int i2);

    @ByVal
    public static native CvScalar cvGet3D(@Const opencv_core.CvArr cvArr, int i, int i2, int i3);

    public static native CvMat cvGetCol(@Const opencv_core.CvArr cvArr, CvMat cvMat, int i);

    public static native CvMat cvGetCols(@Const opencv_core.CvArr cvArr, CvMat cvMat, int i, int i2);

    public static native CvMat cvGetDiag(@Const opencv_core.CvArr cvArr, CvMat cvMat);

    public static native CvMat cvGetDiag(@Const opencv_core.CvArr cvArr, CvMat cvMat, int i);

    public static native int cvGetDimSize(@Const opencv_core.CvArr cvArr, int i);

    public static native int cvGetDims(@Const opencv_core.CvArr cvArr);

    public static native int cvGetDims(@Const opencv_core.CvArr cvArr, IntBuffer intBuffer);

    public static native int cvGetDims(@Const opencv_core.CvArr cvArr, IntPointer intPointer);

    public static native int cvGetDims(@Const opencv_core.CvArr cvArr, int[] iArr);

    public static native int cvGetElemType(@Const opencv_core.CvArr cvArr);

    public static native int cvGetErrInfo(@ByPtrPtr @Cast({"const char**"}) ByteBuffer byteBuffer, @ByPtrPtr @Cast({"const char**"}) ByteBuffer byteBuffer2, @ByPtrPtr @Cast({"const char**"}) ByteBuffer byteBuffer3, IntBuffer intBuffer);

    public static native int cvGetErrInfo(@ByPtrPtr @Cast({"const char**"}) BytePointer bytePointer, @ByPtrPtr @Cast({"const char**"}) BytePointer bytePointer2, @ByPtrPtr @Cast({"const char**"}) BytePointer bytePointer3, IntPointer intPointer);

    public static native int cvGetErrInfo(@Cast({"const char**"}) PointerPointer pointerPointer, @Cast({"const char**"}) PointerPointer pointerPointer2, @Cast({"const char**"}) PointerPointer pointerPointer3, IntPointer intPointer);

    public static native int cvGetErrInfo(@ByPtrPtr @Cast({"const char**"}) byte[] bArr, @ByPtrPtr @Cast({"const char**"}) byte[] bArr2, @ByPtrPtr @Cast({"const char**"}) byte[] bArr3, int[] iArr);

    public static native int cvGetErrMode();

    public static native int cvGetErrStatus();

    public static native IplImage cvGetImage(@Const opencv_core.CvArr cvArr, IplImage iplImage);

    public static native int cvGetImageCOI(@Const IplImage iplImage);

    @ByVal
    public static native CvRect cvGetImageROI(@Const IplImage iplImage);

    public static native CvMat cvGetMat(@Const opencv_core.CvArr cvArr, CvMat cvMat);

    public static native CvMat cvGetMat(@Const opencv_core.CvArr cvArr, CvMat cvMat, IntBuffer intBuffer, int i);

    public static native CvMat cvGetMat(@Const opencv_core.CvArr cvArr, CvMat cvMat, IntPointer intPointer, int i);

    public static native CvMat cvGetMat(@Const opencv_core.CvArr cvArr, CvMat cvMat, int[] iArr, int i);

    @ByVal
    public static native CvScalar cvGetND(@Const opencv_core.CvArr cvArr, @Const IntBuffer intBuffer);

    @ByVal
    public static native CvScalar cvGetND(@Const opencv_core.CvArr cvArr, @Const IntPointer intPointer);

    @ByVal
    public static native CvScalar cvGetND(@Const opencv_core.CvArr cvArr, @Const int[] iArr);

    public static native CvSparseNode cvGetNextSparseNode(CvSparseMatIterator cvSparseMatIterator);

    public static native int cvGetNumThreads();

    public static native int cvGetOptimalDFTSize(int i);

    public static native void cvGetRawData(@Const opencv_core.CvArr cvArr, @ByPtrPtr @Cast({"uchar**"}) ByteBuffer byteBuffer);

    public static native void cvGetRawData(@Const opencv_core.CvArr cvArr, @ByPtrPtr @Cast({"uchar**"}) ByteBuffer byteBuffer, IntBuffer intBuffer, CvSize cvSize);

    public static native void cvGetRawData(@Const opencv_core.CvArr cvArr, @ByPtrPtr @Cast({"uchar**"}) BytePointer bytePointer);

    public static native void cvGetRawData(@Const opencv_core.CvArr cvArr, @ByPtrPtr @Cast({"uchar**"}) BytePointer bytePointer, IntPointer intPointer, CvSize cvSize);

    public static native void cvGetRawData(@Const opencv_core.CvArr cvArr, @Cast({"uchar**"}) PointerPointer pointerPointer, IntPointer intPointer, CvSize cvSize);

    public static native void cvGetRawData(@Const opencv_core.CvArr cvArr, @ByPtrPtr @Cast({"uchar**"}) byte[] bArr);

    public static native void cvGetRawData(@Const opencv_core.CvArr cvArr, @ByPtrPtr @Cast({"uchar**"}) byte[] bArr, int[] iArr, CvSize cvSize);

    public static native double cvGetReal1D(@Const opencv_core.CvArr cvArr, int i);

    public static native double cvGetReal2D(@Const opencv_core.CvArr cvArr, int i, int i2);

    public static native double cvGetReal3D(@Const opencv_core.CvArr cvArr, int i, int i2, int i3);

    public static native double cvGetRealND(@Const opencv_core.CvArr cvArr, @Const IntBuffer intBuffer);

    public static native double cvGetRealND(@Const opencv_core.CvArr cvArr, @Const IntPointer intPointer);

    public static native double cvGetRealND(@Const opencv_core.CvArr cvArr, @Const int[] iArr);

    public static native CvMat cvGetRow(@Const opencv_core.CvArr cvArr, CvMat cvMat, int i);

    public static native CvMat cvGetRows(@Const opencv_core.CvArr cvArr, CvMat cvMat, int i, int i2);

    public static native CvMat cvGetRows(@Const opencv_core.CvArr cvArr, CvMat cvMat, int i, int i2, int i3);

    @Cast({"schar*"})
    public static native BytePointer cvGetSeqElem(@Const CvSeq cvSeq, int i);

    public static native int cvGetSeqReaderPos(CvSeqReader cvSeqReader);

    public static native CvSetElem cvGetSetElem(@Const CvSet cvSet, int i);

    @ByVal
    public static native CvSize cvGetSize(@Const opencv_core.CvArr cvArr);

    public static native CvMat cvGetSubArr(opencv_core.CvArr cvArr, CvMat cvMat, @ByVal CvRect cvRect);

    public static native CvMat cvGetSubRect(@Const opencv_core.CvArr cvArr, CvMat cvMat, @ByVal CvRect cvRect);

    public static native int cvGetThreadNum();

    @Cast({"int64"})
    public static native long cvGetTickCount();

    public static native double cvGetTickFrequency();

    public static native int cvGraphAddEdge(CvGraph cvGraph, int i, int i2);

    public static native int cvGraphAddEdge(CvGraph cvGraph, int i, int i2, @Const CvGraphEdge cvGraphEdge, @Cast({"CvGraphEdge**"}) PointerPointer pointerPointer);

    public static native int cvGraphAddEdge(CvGraph cvGraph, int i, int i2, @Const CvGraphEdge cvGraphEdge, @ByPtrPtr CvGraphEdge cvGraphEdge2);

    public static native int cvGraphAddEdgeByPtr(CvGraph cvGraph, CvGraphVtx cvGraphVtx, CvGraphVtx cvGraphVtx2);

    public static native int cvGraphAddEdgeByPtr(CvGraph cvGraph, CvGraphVtx cvGraphVtx, CvGraphVtx cvGraphVtx2, @Const CvGraphEdge cvGraphEdge, @Cast({"CvGraphEdge**"}) PointerPointer pointerPointer);

    public static native int cvGraphAddEdgeByPtr(CvGraph cvGraph, CvGraphVtx cvGraphVtx, CvGraphVtx cvGraphVtx2, @Const CvGraphEdge cvGraphEdge, @ByPtrPtr CvGraphEdge cvGraphEdge2);

    public static native int cvGraphAddVtx(CvGraph cvGraph);

    public static native int cvGraphAddVtx(CvGraph cvGraph, @Const CvGraphVtx cvGraphVtx, @Cast({"CvGraphVtx**"}) PointerPointer pointerPointer);

    public static native int cvGraphAddVtx(CvGraph cvGraph, @Const CvGraphVtx cvGraphVtx, @ByPtrPtr CvGraphVtx cvGraphVtx2);

    public static native CvGraphEdge cvGraphFindEdge(CvGraph cvGraph, int i, int i2);

    public static native CvGraphEdge cvGraphFindEdgeByPtr(CvGraph cvGraph, CvGraphVtx cvGraphVtx, CvGraphVtx cvGraphVtx2);

    public static native void cvGraphRemoveEdge(CvGraph cvGraph, int i, int i2);

    public static native void cvGraphRemoveEdgeByPtr(CvGraph cvGraph, CvGraphVtx cvGraphVtx, CvGraphVtx cvGraphVtx2);

    public static native int cvGraphRemoveVtx(CvGraph cvGraph, int i);

    public static native int cvGraphRemoveVtxByPtr(CvGraph cvGraph, CvGraphVtx cvGraphVtx);

    public static native int cvGraphVtxDegree(@Const CvGraph cvGraph, int i);

    public static native int cvGraphVtxDegreeByPtr(@Const CvGraph cvGraph, @Const CvGraphVtx cvGraphVtx);

    public static native int cvGuiBoxReport(int i, String str, String str2, String str3, int i2, Pointer pointer);

    public static native int cvGuiBoxReport(int i, @Cast({"const char*"}) BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"const char*"}) BytePointer bytePointer3, int i2, Pointer pointer);

    public static native void cvInRange(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4);

    public static native void cvInRangeS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, @ByVal CvScalar cvScalar2, opencv_core.CvArr cvArr2);

    public static native int cvIncRefData(opencv_core.CvArr cvArr);

    public static native IplImage cvInitImageHeader(IplImage iplImage, @ByVal CvSize cvSize, int i, int i2);

    public static native IplImage cvInitImageHeader(IplImage iplImage, @ByVal CvSize cvSize, int i, int i2, int i3, int i4);

    public static native CvMat cvInitMatHeader(CvMat cvMat, int i, int i2, int i3);

    public static native CvMat cvInitMatHeader(CvMat cvMat, int i, int i2, int i3, Pointer pointer, int i4);

    public static native CvMatND cvInitMatNDHeader(CvMatND cvMatND, int i, @Const IntBuffer intBuffer, int i2);

    public static native CvMatND cvInitMatNDHeader(CvMatND cvMatND, int i, @Const IntBuffer intBuffer, int i2, Pointer pointer);

    public static native CvMatND cvInitMatNDHeader(CvMatND cvMatND, int i, @Const IntPointer intPointer, int i2);

    public static native CvMatND cvInitMatNDHeader(CvMatND cvMatND, int i, @Const IntPointer intPointer, int i2, Pointer pointer);

    public static native CvMatND cvInitMatNDHeader(CvMatND cvMatND, int i, @Const int[] iArr, int i2);

    public static native CvMatND cvInitMatNDHeader(CvMatND cvMatND, int i, @Const int[] iArr, int i2, Pointer pointer);

    public static native int cvInitNArrayIterator(int i, @Cast({"CvArr**"}) PointerPointer pointerPointer, @Const opencv_core.CvArr cvArr, CvMatND cvMatND, CvNArrayIterator cvNArrayIterator, int i2);

    public static native int cvInitNArrayIterator(int i, @ByPtrPtr opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, CvMatND cvMatND, CvNArrayIterator cvNArrayIterator);

    public static native int cvInitNArrayIterator(int i, @ByPtrPtr opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, CvMatND cvMatND, CvNArrayIterator cvNArrayIterator, int i2);

    public static native CvSparseNode cvInitSparseMatIterator(@Const CvSparseMat cvSparseMat, CvSparseMatIterator cvSparseMatIterator);

    public static native void cvInitTreeNodeIterator(CvTreeNodeIterator cvTreeNodeIterator, @Const Pointer pointer, int i);

    public static native void cvInsertNodeIntoTree(Pointer pointer, Pointer pointer2, Pointer pointer3);

    public static native void cvInv(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i);

    public static native double cvInvert(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native double cvInvert(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i);

    public static native int cvIplDepth(int i);

    @ByVal
    public static native IplImage cvIplImage();

    @ByVal
    public static native IplImage cvIplImage(@ByRef @Const Mat mat);

    public static native int cvIsInf(double d);

    public static native int cvIsInf(float f);

    public static native int cvIsNaN(double d);

    public static native int cvIsNaN(float f);

    public static native int cvKMeans2(@Const opencv_core.CvArr cvArr, int i, opencv_core.CvArr cvArr2, @ByVal CvTermCriteria cvTermCriteria);

    public static native int cvKMeans2(@Const opencv_core.CvArr cvArr, int i, opencv_core.CvArr cvArr2, @ByVal CvTermCriteria cvTermCriteria, int i2, @Cast({"CvRNG*"}) LongBuffer longBuffer, int i3, opencv_core.CvArr cvArr3, DoubleBuffer doubleBuffer);

    public static native int cvKMeans2(@Const opencv_core.CvArr cvArr, int i, opencv_core.CvArr cvArr2, @ByVal CvTermCriteria cvTermCriteria, int i2, @Cast({"CvRNG*"}) LongPointer longPointer, int i3, opencv_core.CvArr cvArr3, DoublePointer doublePointer);

    public static native int cvKMeans2(@Const opencv_core.CvArr cvArr, int i, opencv_core.CvArr cvArr2, @ByVal CvTermCriteria cvTermCriteria, int i2, @Cast({"CvRNG*"}) long[] jArr, int i3, opencv_core.CvArr cvArr3, double[] dArr);

    public static native void cvLUT(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3);

    public static native void cvLog(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native double cvMahalanobis(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3);

    public static native double cvMahalonobis(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native CvSeq cvMakeSeqHeaderForArray(int i, int i2, int i3, Pointer pointer, int i4, CvSeq cvSeq, CvSeqBlock cvSeqBlock);

    @ByVal
    public static native CvMat cvMat();

    @ByVal
    public static native CvMat cvMat(int i, int i2, int i3);

    @ByVal
    public static native CvMat cvMat(int i, int i2, int i3, Pointer pointer);

    @ByVal
    public static native CvMat cvMat(@ByRef @Const CvMat cvMat);

    @ByVal
    public static native CvMat cvMat(@ByRef @Const Mat mat);

    public static native void cvMatMul(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvMatMulAdd(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4);

    public static native void cvMatMulAddEx(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, double d, opencv_core.CvArr cvArr3, double d2, opencv_core.CvArr cvArr4, int i);

    public static native void cvMatMulAddS(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, CvMat cvMat, CvMat cvMat2);

    @ByVal
    public static native CvMatND cvMatND();

    @ByVal
    public static native CvMatND cvMatND(@ByRef @Const Mat mat);

    public static native void cvMax(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvMaxS(@Const opencv_core.CvArr cvArr, double d, opencv_core.CvArr cvArr2);

    public static native Pointer cvMemStorageAlloc(CvMemStorage cvMemStorage, @Cast({"size_t"}) long j);

    public static native void cvMerge(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3, @Const opencv_core.CvArr cvArr4, opencv_core.CvArr cvArr5);

    public static native void cvMin(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvMinMaxLoc(@Const opencv_core.CvArr cvArr, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2);

    public static native void cvMinMaxLoc(@Const opencv_core.CvArr cvArr, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, @Cast({"CvPoint*"}) IntBuffer intBuffer, @Cast({"CvPoint*"}) IntBuffer intBuffer2, @Const opencv_core.CvArr cvArr2);

    public static native void cvMinMaxLoc(@Const opencv_core.CvArr cvArr, DoublePointer doublePointer, DoublePointer doublePointer2);

    public static native void cvMinMaxLoc(@Const opencv_core.CvArr cvArr, DoublePointer doublePointer, DoublePointer doublePointer2, CvPoint cvPoint, CvPoint cvPoint2, @Const opencv_core.CvArr cvArr2);

    public static native void cvMinMaxLoc(@Const opencv_core.CvArr cvArr, double[] dArr, double[] dArr2);

    public static native void cvMinMaxLoc(@Const opencv_core.CvArr cvArr, double[] dArr, double[] dArr2, @Cast({"CvPoint*"}) int[] iArr, @Cast({"CvPoint*"}) int[] iArr2, @Const opencv_core.CvArr cvArr2);

    public static native void cvMinS(@Const opencv_core.CvArr cvArr, double d, opencv_core.CvArr cvArr2);

    public static native void cvMirror(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i);

    public static native void cvMixChannels(@Cast({"const CvArr**"}) PointerPointer pointerPointer, int i, @Cast({"CvArr**"}) PointerPointer pointerPointer2, int i2, @Const IntPointer intPointer, int i3);

    public static native void cvMixChannels(@ByPtrPtr @Const opencv_core.CvArr cvArr, int i, @ByPtrPtr opencv_core.CvArr cvArr2, int i2, @Const IntBuffer intBuffer, int i3);

    public static native void cvMixChannels(@ByPtrPtr @Const opencv_core.CvArr cvArr, int i, @ByPtrPtr opencv_core.CvArr cvArr2, int i2, @Const IntPointer intPointer, int i3);

    public static native void cvMixChannels(@ByPtrPtr @Const opencv_core.CvArr cvArr, int i, @ByPtrPtr opencv_core.CvArr cvArr2, int i2, @Const int[] iArr, int i3);

    public static native void cvMul(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvMul(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, double d);

    public static native void cvMulSpectrums(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, int i);

    public static native void cvMulTransposed(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i);

    public static native void cvMulTransposed(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i, @Const opencv_core.CvArr cvArr3, double d);

    public static native int cvNextGraphItem(CvGraphScanner cvGraphScanner);

    public static native int cvNextNArraySlice(CvNArrayIterator cvNArrayIterator);

    public static native Pointer cvNextTreeNode(CvTreeNodeIterator cvTreeNodeIterator);

    public static native double cvNorm(@Const opencv_core.CvArr cvArr);

    public static native double cvNorm(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, int i, @Const opencv_core.CvArr cvArr3);

    public static native void cvNormalize(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native void cvNormalize(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, double d, double d2, int i, @Const opencv_core.CvArr cvArr3);

    public static native void cvNot(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native int cvNulDevReport(int i, String str, String str2, String str3, int i2, Pointer pointer);

    public static native int cvNulDevReport(int i, @Cast({"const char*"}) BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"const char*"}) BytePointer bytePointer3, int i2, Pointer pointer);

    public static native void cvOr(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvOr(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, @Const opencv_core.CvArr cvArr4);

    public static native void cvOrS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2);

    public static native void cvOrS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3);

    public static native void cvPerspectiveTransform(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, @Const CvMat cvMat);

    @ByVal
    public static native CvPoint cvPoint(int i, int i2);

    @ByVal
    public static native CvPoint cvPoint(@ByRef @Const Point point);

    @ByVal
    public static native CvPoint2D32f cvPoint2D32f(double d, double d2);

    @ByVal
    public static native CvPoint2D64f cvPoint2D64f(double d, double d2);

    @ByVal
    public static native CvPoint3D32f cvPoint3D32f(double d, double d2, double d3);

    @ByVal
    public static native CvPoint3D64f cvPoint3D64f(double d, double d2, double d3);

    @Cast({"CvPoint*"})
    @ByVal
    public static native IntBuffer cvPointFrom32f(@Cast({"CvPoint2D32f*"}) @ByVal FloatBuffer floatBuffer);

    @ByVal
    public static native CvPoint cvPointFrom32f(@ByVal CvPoint2D32f cvPoint2D32f);

    @Cast({"CvPoint*"})
    @ByVal
    public static native int[] cvPointFrom32f(@Cast({"CvPoint2D32f*"}) @ByVal float[] fArr);

    @Cast({"CvPoint2D32f*"})
    @ByVal
    public static native FloatBuffer cvPointTo32f(@Cast({"CvPoint*"}) @ByVal IntBuffer intBuffer);

    @ByVal
    public static native CvPoint2D32f cvPointTo32f(@ByVal CvPoint cvPoint);

    @Cast({"CvPoint2D32f*"})
    @ByVal
    public static native float[] cvPointTo32f(@Cast({"CvPoint*"}) @ByVal int[] iArr);

    public static native void cvPolarToCart(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4);

    public static native void cvPolarToCart(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4, int i);

    public static native void cvPow(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, double d);

    public static native Pointer cvPrevTreeNode(CvTreeNodeIterator cvTreeNodeIterator);

    public static native void cvProjectPCA(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4);

    @Cast({"uchar*"})
    public static native ByteBuffer cvPtr1D(@Const opencv_core.CvArr cvArr, int i, IntBuffer intBuffer);

    @Cast({"uchar*"})
    public static native BytePointer cvPtr1D(@Const opencv_core.CvArr cvArr, int i);

    @Cast({"uchar*"})
    public static native BytePointer cvPtr1D(@Const opencv_core.CvArr cvArr, int i, IntPointer intPointer);

    @Cast({"uchar*"})
    public static native byte[] cvPtr1D(@Const opencv_core.CvArr cvArr, int i, int[] iArr);

    @Cast({"uchar*"})
    public static native ByteBuffer cvPtr2D(@Const opencv_core.CvArr cvArr, int i, int i2, IntBuffer intBuffer);

    @Cast({"uchar*"})
    public static native BytePointer cvPtr2D(@Const opencv_core.CvArr cvArr, int i, int i2);

    @Cast({"uchar*"})
    public static native BytePointer cvPtr2D(@Const opencv_core.CvArr cvArr, int i, int i2, IntPointer intPointer);

    @Cast({"uchar*"})
    public static native byte[] cvPtr2D(@Const opencv_core.CvArr cvArr, int i, int i2, int[] iArr);

    @Cast({"uchar*"})
    public static native ByteBuffer cvPtr3D(@Const opencv_core.CvArr cvArr, int i, int i2, int i3, IntBuffer intBuffer);

    @Cast({"uchar*"})
    public static native BytePointer cvPtr3D(@Const opencv_core.CvArr cvArr, int i, int i2, int i3);

    @Cast({"uchar*"})
    public static native BytePointer cvPtr3D(@Const opencv_core.CvArr cvArr, int i, int i2, int i3, IntPointer intPointer);

    @Cast({"uchar*"})
    public static native byte[] cvPtr3D(@Const opencv_core.CvArr cvArr, int i, int i2, int i3, int[] iArr);

    @Cast({"uchar*"})
    public static native ByteBuffer cvPtrND(@Const opencv_core.CvArr cvArr, @Const IntBuffer intBuffer);

    @Cast({"uchar*"})
    public static native ByteBuffer cvPtrND(@Const opencv_core.CvArr cvArr, @Const IntBuffer intBuffer, IntBuffer intBuffer2, int i, @Cast({"unsigned*"}) IntBuffer intBuffer3);

    @Cast({"uchar*"})
    public static native BytePointer cvPtrND(@Const opencv_core.CvArr cvArr, @Const IntPointer intPointer);

    @Cast({"uchar*"})
    public static native BytePointer cvPtrND(@Const opencv_core.CvArr cvArr, @Const IntPointer intPointer, IntPointer intPointer2, int i, @Cast({"unsigned*"}) IntPointer intPointer3);

    @Cast({"uchar*"})
    public static native byte[] cvPtrND(@Const opencv_core.CvArr cvArr, @Const int[] iArr);

    @Cast({"uchar*"})
    public static native byte[] cvPtrND(@Const opencv_core.CvArr cvArr, @Const int[] iArr, int[] iArr2, int i, @Cast({"unsigned*"}) int[] iArr3);

    @Cast({"CvRNG"})
    public static native long cvRNG();

    @Cast({"CvRNG"})
    public static native long cvRNG(@Cast({"int64"}) long j);

    @ByVal
    public static native CvRect cvROIToRect(@ByVal IplROI iplROI);

    public static native void cvRandArr(@Cast({"CvRNG*"}) LongBuffer longBuffer, opencv_core.CvArr cvArr, int i, @ByVal CvScalar cvScalar, @ByVal CvScalar cvScalar2);

    public static native void cvRandArr(@Cast({"CvRNG*"}) LongPointer longPointer, opencv_core.CvArr cvArr, int i, @ByVal CvScalar cvScalar, @ByVal CvScalar cvScalar2);

    public static native void cvRandArr(@Cast({"CvRNG*"}) long[] jArr, opencv_core.CvArr cvArr, int i, @ByVal CvScalar cvScalar, @ByVal CvScalar cvScalar2);

    @Cast({"unsigned"})
    public static native int cvRandInt(@Cast({"CvRNG*"}) LongBuffer longBuffer);

    @Cast({"unsigned"})
    public static native int cvRandInt(@Cast({"CvRNG*"}) LongPointer longPointer);

    @Cast({"unsigned"})
    public static native int cvRandInt(@Cast({"CvRNG*"}) long[] jArr);

    public static native double cvRandReal(@Cast({"CvRNG*"}) LongBuffer longBuffer);

    public static native double cvRandReal(@Cast({"CvRNG*"}) LongPointer longPointer);

    public static native double cvRandReal(@Cast({"CvRNG*"}) long[] jArr);

    public static native void cvRandShuffle(opencv_core.CvArr cvArr, @Cast({"CvRNG*"}) LongBuffer longBuffer);

    public static native void cvRandShuffle(opencv_core.CvArr cvArr, @Cast({"CvRNG*"}) LongBuffer longBuffer, double d);

    public static native void cvRandShuffle(opencv_core.CvArr cvArr, @Cast({"CvRNG*"}) LongPointer longPointer);

    public static native void cvRandShuffle(opencv_core.CvArr cvArr, @Cast({"CvRNG*"}) LongPointer longPointer, double d);

    public static native void cvRandShuffle(opencv_core.CvArr cvArr, @Cast({"CvRNG*"}) long[] jArr);

    public static native void cvRandShuffle(opencv_core.CvArr cvArr, @Cast({"CvRNG*"}) long[] jArr, double d);

    public static native opencv_core.CvArr cvRange(opencv_core.CvArr cvArr, double d, double d2);

    public static native void cvRawDataToScalar(@Const Pointer pointer, int i, CvScalar cvScalar);

    @ByVal
    public static native CvScalar cvRealScalar(double d);

    @ByVal
    public static native CvRect cvRect(int i, int i2, int i3, int i4);

    @ByVal
    public static native CvRect cvRect(@ByRef @Const Rect rect);

    @ByVal
    public static native IplROI cvRectToROI(@ByVal CvRect cvRect, int i);

    public static native CvErrorCallback cvRedirectError(CvErrorCallback cvErrorCallback);

    public static native CvErrorCallback cvRedirectError(CvErrorCallback cvErrorCallback, Pointer pointer, @ByPtrPtr @Cast({"void**"}) Pointer pointer2);

    public static native CvErrorCallback cvRedirectError(CvErrorCallback cvErrorCallback, Pointer pointer, @Cast({"void**"}) PointerPointer pointerPointer);

    public static native void cvReduce(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native void cvReduce(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, int i, int i2);

    public static native void cvRelease(@ByPtrPtr @Cast({"void**"}) Pointer pointer);

    public static native void cvRelease(@Cast({"void**"}) PointerPointer pointerPointer);

    public static native void cvReleaseData(opencv_core.CvArr cvArr);

    public static native void cvReleaseGraphScanner(@Cast({"CvGraphScanner**"}) PointerPointer pointerPointer);

    public static native void cvReleaseGraphScanner(@ByPtrPtr CvGraphScanner cvGraphScanner);

    public static native void cvReleaseImage(@Cast({"IplImage**"}) PointerPointer pointerPointer);

    public static native void cvReleaseImage(@ByPtrPtr IplImage iplImage);

    public static native void cvReleaseImageHeader(@Cast({"IplImage**"}) PointerPointer pointerPointer);

    public static native void cvReleaseImageHeader(@ByPtrPtr IplImage iplImage);

    public static native void cvReleaseMat(@Cast({"CvMat**"}) PointerPointer pointerPointer);

    public static native void cvReleaseMat(@ByPtrPtr CvMat cvMat);

    public static native void cvReleaseMatND(@Cast({"CvMatND**"}) PointerPointer pointerPointer);

    public static native void cvReleaseMatND(@ByPtrPtr CvMatND cvMatND);

    public static native void cvReleaseMemStorage(@Cast({"CvMemStorage**"}) PointerPointer pointerPointer);

    public static native void cvReleaseMemStorage(@ByPtrPtr CvMemStorage cvMemStorage);

    public static native void cvReleaseSparseMat(@Cast({"CvSparseMat**"}) PointerPointer pointerPointer);

    public static native void cvReleaseSparseMat(@ByPtrPtr CvSparseMat cvSparseMat);

    public static native void cvRemoveNodeFromTree(Pointer pointer, Pointer pointer2);

    public static native void cvRepeat(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native void cvResetImageROI(IplImage iplImage);

    public static native CvMat cvReshape(@Const opencv_core.CvArr cvArr, CvMat cvMat, int i);

    public static native CvMat cvReshape(@Const opencv_core.CvArr cvArr, CvMat cvMat, int i, int i2);

    public static native opencv_core.CvArr cvReshapeMatND(@Const opencv_core.CvArr cvArr, int i, opencv_core.CvArr cvArr2, int i2, int i3, IntBuffer intBuffer);

    public static native opencv_core.CvArr cvReshapeMatND(@Const opencv_core.CvArr cvArr, int i, opencv_core.CvArr cvArr2, int i2, int i3, IntPointer intPointer);

    public static native opencv_core.CvArr cvReshapeMatND(@Const opencv_core.CvArr cvArr, int i, opencv_core.CvArr cvArr2, int i2, int i3, int[] iArr);

    public static native void cvRestoreMemStoragePos(CvMemStorage cvMemStorage, CvMemStoragePos cvMemStoragePos);

    public static native int cvRound(double d);

    public static native int cvRound(float f);

    public static native int cvRound(int i);

    public static native void cvSVBkSb(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3, @Const opencv_core.CvArr cvArr4, opencv_core.CvArr cvArr5, int i);

    public static native void cvSVD(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native void cvSVD(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4, int i);

    public static native void cvSaveMemStoragePos(@Const CvMemStorage cvMemStorage, CvMemStoragePos cvMemStoragePos);

    @ByVal
    public static native CvScalar cvScalar();

    @ByVal
    public static native CvScalar cvScalar(double d);

    @ByVal
    public static native CvScalar cvScalar(double d, double d2, double d3, double d4);

    @ByVal
    public static native CvScalar cvScalar(@ByRef @Const Scalar scalar);

    @ByVal
    public static native CvScalar cvScalarAll(double d);

    public static native void cvScalarToRawData(@Const CvScalar cvScalar, Pointer pointer, int i);

    public static native void cvScalarToRawData(@Const CvScalar cvScalar, Pointer pointer, int i, int i2);

    public static native void cvScale(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, double d, double d2);

    public static native void cvScaleAdd(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native int cvSeqElemIdx(@Const CvSeq cvSeq, @Const Pointer pointer);

    public static native int cvSeqElemIdx(@Const CvSeq cvSeq, @Const Pointer pointer, @Cast({"CvSeqBlock**"}) PointerPointer pointerPointer);

    public static native int cvSeqElemIdx(@Const CvSeq cvSeq, @Const Pointer pointer, @ByPtrPtr CvSeqBlock cvSeqBlock);

    @Cast({"schar*"})
    public static native BytePointer cvSeqInsert(CvSeq cvSeq, int i);

    @Cast({"schar*"})
    public static native BytePointer cvSeqInsert(CvSeq cvSeq, int i, @Const Pointer pointer);

    public static native void cvSeqInsertSlice(CvSeq cvSeq, int i, @Const opencv_core.CvArr cvArr);

    public static native void cvSeqInvert(CvSeq cvSeq);

    public static native int cvSeqPartition(@Const CvSeq cvSeq, CvMemStorage cvMemStorage, @Cast({"CvSeq**"}) PointerPointer pointerPointer, CvCmpFunc cvCmpFunc, Pointer pointer);

    public static native int cvSeqPartition(@Const CvSeq cvSeq, CvMemStorage cvMemStorage, @ByPtrPtr CvSeq cvSeq2, CvCmpFunc cvCmpFunc, Pointer pointer);

    public static native void cvSeqPop(CvSeq cvSeq);

    public static native void cvSeqPop(CvSeq cvSeq, Pointer pointer);

    public static native void cvSeqPopFront(CvSeq cvSeq);

    public static native void cvSeqPopFront(CvSeq cvSeq, Pointer pointer);

    public static native void cvSeqPopMulti(CvSeq cvSeq, Pointer pointer, int i);

    public static native void cvSeqPopMulti(CvSeq cvSeq, Pointer pointer, int i, int i2);

    @Cast({"schar*"})
    public static native BytePointer cvSeqPush(CvSeq cvSeq);

    @Cast({"schar*"})
    public static native BytePointer cvSeqPush(CvSeq cvSeq, @Const Pointer pointer);

    @Cast({"schar*"})
    public static native BytePointer cvSeqPushFront(CvSeq cvSeq);

    @Cast({"schar*"})
    public static native BytePointer cvSeqPushFront(CvSeq cvSeq, @Const Pointer pointer);

    public static native void cvSeqPushMulti(CvSeq cvSeq, @Const Pointer pointer, int i);

    public static native void cvSeqPushMulti(CvSeq cvSeq, @Const Pointer pointer, int i, int i2);

    public static native void cvSeqRemove(CvSeq cvSeq, int i);

    public static native void cvSeqRemoveSlice(CvSeq cvSeq, @ByVal CvSlice cvSlice);

    @Cast({"schar*"})
    public static native ByteBuffer cvSeqSearch(CvSeq cvSeq, @Const Pointer pointer, CvCmpFunc cvCmpFunc, int i, IntBuffer intBuffer);

    @Cast({"schar*"})
    public static native ByteBuffer cvSeqSearch(CvSeq cvSeq, @Const Pointer pointer, CvCmpFunc cvCmpFunc, int i, IntBuffer intBuffer, Pointer pointer2);

    @Cast({"schar*"})
    public static native BytePointer cvSeqSearch(CvSeq cvSeq, @Const Pointer pointer, CvCmpFunc cvCmpFunc, int i, IntPointer intPointer);

    @Cast({"schar*"})
    public static native BytePointer cvSeqSearch(CvSeq cvSeq, @Const Pointer pointer, CvCmpFunc cvCmpFunc, int i, IntPointer intPointer, Pointer pointer2);

    @Cast({"schar*"})
    public static native byte[] cvSeqSearch(CvSeq cvSeq, @Const Pointer pointer, CvCmpFunc cvCmpFunc, int i, int[] iArr);

    @Cast({"schar*"})
    public static native byte[] cvSeqSearch(CvSeq cvSeq, @Const Pointer pointer, CvCmpFunc cvCmpFunc, int i, int[] iArr, Pointer pointer2);

    public static native CvSeq cvSeqSlice(@Const CvSeq cvSeq, @ByVal CvSlice cvSlice);

    public static native CvSeq cvSeqSlice(@Const CvSeq cvSeq, @ByVal CvSlice cvSlice, CvMemStorage cvMemStorage, int i);

    public static native void cvSeqSort(CvSeq cvSeq, CvCmpFunc cvCmpFunc);

    public static native void cvSeqSort(CvSeq cvSeq, CvCmpFunc cvCmpFunc, Pointer pointer);

    public static native void cvSet(opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar);

    public static native void cvSet(opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, @Const opencv_core.CvArr cvArr2);

    public static native void cvSet1D(opencv_core.CvArr cvArr, int i, @ByVal CvScalar cvScalar);

    public static native void cvSet2D(opencv_core.CvArr cvArr, int i, int i2, @ByVal CvScalar cvScalar);

    public static native void cvSet3D(opencv_core.CvArr cvArr, int i, int i2, int i3, @ByVal CvScalar cvScalar);

    public static native int cvSetAdd(CvSet cvSet);

    public static native int cvSetAdd(CvSet cvSet, CvSetElem cvSetElem, @Cast({"CvSetElem**"}) PointerPointer pointerPointer);

    public static native int cvSetAdd(CvSet cvSet, CvSetElem cvSetElem, @ByPtrPtr CvSetElem cvSetElem2);

    public static native void cvSetData(opencv_core.CvArr cvArr, Pointer pointer, int i);

    public static native int cvSetErrMode(int i);

    public static native void cvSetErrStatus(int i);

    public static native void cvSetIPLAllocators(Cv_iplCreateImageHeader cv_iplCreateImageHeader, Cv_iplAllocateImageData cv_iplAllocateImageData, Cv_iplDeallocate cv_iplDeallocate, Cv_iplCreateROI cv_iplCreateROI, Cv_iplCloneImage cv_iplCloneImage);

    public static native void cvSetIdentity(opencv_core.CvArr cvArr);

    public static native void cvSetIdentity(opencv_core.CvArr cvArr, @ByVal(nullValue = "CvScalar(cvRealScalar(1))") CvScalar cvScalar);

    public static native void cvSetImageCOI(IplImage iplImage, int i);

    public static native void cvSetImageROI(IplImage iplImage, @ByVal CvRect cvRect);

    public static native void cvSetND(opencv_core.CvArr cvArr, @Const IntBuffer intBuffer, @ByVal CvScalar cvScalar);

    public static native void cvSetND(opencv_core.CvArr cvArr, @Const IntPointer intPointer, @ByVal CvScalar cvScalar);

    public static native void cvSetND(opencv_core.CvArr cvArr, @Const int[] iArr, @ByVal CvScalar cvScalar);

    public static native CvSetElem cvSetNew(CvSet cvSet);

    public static native void cvSetNumThreads();

    public static native void cvSetNumThreads(int i);

    public static native void cvSetReal1D(opencv_core.CvArr cvArr, int i, double d);

    public static native void cvSetReal2D(opencv_core.CvArr cvArr, int i, int i2, double d);

    public static native void cvSetReal3D(opencv_core.CvArr cvArr, int i, int i2, int i3, double d);

    public static native void cvSetRealND(opencv_core.CvArr cvArr, @Const IntBuffer intBuffer, double d);

    public static native void cvSetRealND(opencv_core.CvArr cvArr, @Const IntPointer intPointer, double d);

    public static native void cvSetRealND(opencv_core.CvArr cvArr, @Const int[] iArr, double d);

    public static native void cvSetRemove(CvSet cvSet, int i);

    public static native void cvSetRemoveByPtr(CvSet cvSet, Pointer pointer);

    public static native void cvSetSeqBlockSize(CvSeq cvSeq, int i);

    public static native void cvSetSeqReaderPos(CvSeqReader cvSeqReader, int i);

    public static native void cvSetSeqReaderPos(CvSeqReader cvSeqReader, int i, int i2);

    public static native void cvSetZero(opencv_core.CvArr cvArr);

    @ByVal
    public static native CvSize cvSize(int i, int i2);

    @ByVal
    public static native CvSize cvSize(@ByRef @Const Size size);

    @ByVal
    public static native CvSize2D32f cvSize2D32f(double d, double d2);

    @ByVal
    public static native CvSlice cvSlice(int i, int i2);

    @ByVal
    public static native CvSlice cvSlice(@ByRef @Const Range range);

    public static native int cvSliceLength(@ByVal CvSlice cvSlice, @Const CvSeq cvSeq);

    public static native int cvSolve(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native int cvSolve(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, int i);

    public static native int cvSolveCubic(@Const CvMat cvMat, CvMat cvMat2);

    public static native void cvSolvePoly(@Const CvMat cvMat, CvMat cvMat2);

    public static native void cvSolvePoly(@Const CvMat cvMat, CvMat cvMat2, int i, int i2);

    public static native void cvSort(@Const opencv_core.CvArr cvArr);

    public static native void cvSort(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, int i);

    public static native void cvSplit(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, opencv_core.CvArr cvArr4, opencv_core.CvArr cvArr5);

    public static native void cvStartAppendToSeq(CvSeq cvSeq, CvSeqWriter cvSeqWriter);

    public static native void cvStartReadSeq(@Const CvSeq cvSeq, CvSeqReader cvSeqReader);

    public static native void cvStartReadSeq(@Const CvSeq cvSeq, CvSeqReader cvSeqReader, int i);

    public static native void cvStartWriteSeq(int i, int i2, int i3, CvMemStorage cvMemStorage, CvSeqWriter cvSeqWriter);

    public static native int cvStdErrReport(int i, String str, String str2, String str3, int i2, Pointer pointer);

    public static native int cvStdErrReport(int i, @Cast({"const char*"}) BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"const char*"}) BytePointer bytePointer3, int i2, Pointer pointer);

    public static native void cvSub(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvSub(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, @Const opencv_core.CvArr cvArr4);

    public static native void cvSubRS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2);

    public static native void cvSubRS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3);

    public static native void cvSubS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2);

    public static native void cvSubS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3);

    @ByVal
    public static native CvScalar cvSum(@Const opencv_core.CvArr cvArr);

    public static native void cvT(opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    @ByVal
    public static native CvTermCriteria cvTermCriteria(int i, int i2, double d);

    @ByVal
    public static native CvTermCriteria cvTermCriteria(@ByRef @Const TermCriteria termCriteria);

    @ByVal
    public static native CvScalar cvTrace(@Const opencv_core.CvArr cvArr);

    public static native void cvTransform(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, @Const CvMat cvMat);

    public static native void cvTransform(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2, @Const CvMat cvMat, @Const CvMat cvMat2);

    public static native void cvTranspose(@Const opencv_core.CvArr cvArr, opencv_core.CvArr cvArr2);

    public static native CvSeq cvTreeToNodeSeq(@Const Pointer pointer, int i, CvMemStorage cvMemStorage);

    public static native int cvUseOptimized(int i);

    public static native void cvXor(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3);

    public static native void cvXor(@Const opencv_core.CvArr cvArr, @Const opencv_core.CvArr cvArr2, opencv_core.CvArr cvArr3, @Const opencv_core.CvArr cvArr4);

    public static native void cvXorS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2);

    public static native void cvXorS(@Const opencv_core.CvArr cvArr, @ByVal CvScalar cvScalar, opencv_core.CvArr cvArr2, @Const opencv_core.CvArr cvArr3);

    public static native void cvZero(opencv_core.CvArr cvArr);

    @Namespace("cv")
    public static native int cv_abs(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    public static native int cv_abs(@Cast({"ushort"}) short s);

    @Namespace("cv")
    @ByVal
    public static native Mat cvarrToMat(@Const opencv_core.CvArr cvArr);

    @Namespace("cv")
    @ByVal
    public static native Mat cvarrToMat(@Const opencv_core.CvArr cvArr, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i, @Cast({"cv::AutoBuffer<double>*"}) Pointer pointer);

    @Namespace("cv")
    @ByVal
    public static native Mat cvarrToMatND(@Const opencv_core.CvArr cvArr);

    @Namespace("cv")
    @ByVal
    public static native Mat cvarrToMatND(@Const opencv_core.CvArr cvArr, @Cast({"bool"}) boolean z, int i);

    public static native double cvmGet(@Const CvMat cvMat, int i, int i2);

    public static native void cvmSet(CvMat cvMat, int i, int i2, double d);

    @Namespace("cv::hal")
    public static native void cvt16f32f(@Const float16_t float16_t2, FloatBuffer floatBuffer, int i);

    @Namespace("cv::hal")
    public static native void cvt16f32f(@Const float16_t float16_t2, FloatPointer floatPointer, int i);

    @Namespace("cv::hal")
    public static native void cvt16f32f(@Const float16_t float16_t2, float[] fArr, int i);

    @Namespace("cv::hal")
    public static native void cvt32f16f(@Const FloatBuffer floatBuffer, float16_t float16_t2, int i);

    @Namespace("cv::hal")
    public static native void cvt32f16f(@Const FloatPointer floatPointer, float16_t float16_t2, int i);

    @Namespace("cv::hal")
    public static native void cvt32f16f(@Const float[] fArr, float16_t float16_t2, int i);

    @Namespace("cv")
    public static native void dct(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void dct(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native void dct(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void dct(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native void dct(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native void dct(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    public static native double determinant(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native double determinant(@ByVal Mat mat);

    @Namespace("cv")
    public static native double determinant(@ByVal UMat uMat);

    @Namespace("cv::cuda")
    @Cast({"bool"})
    public static native boolean deviceSupports(@Cast({"cv::cuda::FeatureSet"}) int i);

    @Namespace("cv")
    public static native void dft(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void dft(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, int i2);

    @Namespace("cv")
    public static native void dft(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void dft(@ByVal Mat mat, @ByVal Mat mat2, int i, int i2);

    @Namespace("cv")
    public static native void dft(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native void dft(@ByVal UMat uMat, @ByVal UMat uMat2, int i, int i2);

    @Namespace("cv::hal")
    public static native void div16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, IntBuffer intBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, IntPointer intPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, int[] iArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void div8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    public static native int divUp(int i, @Cast({"unsigned int"}) int i2);

    @Namespace("cv")
    @Cast({"size_t"})
    public static native long divUp(@Cast({"size_t"}) long j, @Cast({"unsigned int"}) int i);

    @Namespace("cv")
    @ByVal
    @Name({"operator /"})
    public static native MatExpr divide(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator /"})
    public static native MatExpr divide(double d, @ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @ByVal
    @Name({"operator /"})
    public static native MatExpr divide(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator /"})
    public static native MatExpr divide(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByVal
    @Name({"operator /"})
    public static native MatExpr divide(@ByRef @Const Mat mat, @ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @ByVal
    @Name({"operator /"})
    public static native MatExpr divide(@ByRef @Const MatExpr matExpr, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator /"})
    public static native MatExpr divide(@ByRef @Const MatExpr matExpr, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator /"})
    public static native MatExpr divide(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2);

    @Namespace("cv")
    public static native void divide(double d, @ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void divide(double d, @ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native void divide(double d, @ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void divide(double d, @ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native void divide(double d, @ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native void divide(double d, @ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    public static native void divide(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void divide(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, double d, int i);

    @Namespace("cv")
    public static native void divide(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void divide(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, double d, int i);

    @Namespace("cv")
    public static native void divide(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void divide(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, double d, int i);

    @Namespace("cv")
    @ByRef
    @Name({"operator /="})
    public static native Mat dividePut(@ByRef Mat mat, double d);

    @Namespace("cv")
    @ByRef
    @Name({"operator /="})
    public static native Mat dividePut(@ByRef Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @Name({"randu<double>"})
    public static native double doubleRand();

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean eigen(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean eigen(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") GpuMat gpuMat3);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean eigen(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean eigen(@ByVal Mat mat, @ByVal Mat mat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") Mat mat3);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean eigen(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean eigen(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") UMat uMat3);

    @Namespace("cv")
    public static native void eigenNonSymmetric(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void eigenNonSymmetric(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void eigenNonSymmetric(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv::cuda")
    public static native void ensureSizeIsEnough(int i, int i2, int i3, @ByVal GpuMat gpuMat);

    @Namespace("cv::cuda")
    public static native void ensureSizeIsEnough(int i, int i2, int i3, @ByVal Mat mat);

    @Namespace("cv::cuda")
    public static native void ensureSizeIsEnough(int i, int i2, int i3, @ByVal UMat uMat);

    @Namespace("cv")
    @ByVal
    @Name({"operator =="})
    public static native MatExpr equals(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator =="})
    public static native MatExpr equals(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator =="})
    public static native MatExpr equals(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator =="})
    public static native boolean equals(@Cast({"const cv::AccessFlag"}) int i, int i2);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator =="})
    public static native boolean equals(@ByRef @Const FileNodeIterator fileNodeIterator, @ByRef @Const FileNodeIterator fileNodeIterator2);

    @Namespace("cv::instr")
    @Cast({"bool"})
    @Name({"operator =="})
    public static native boolean equals(@ByRef @Const NodeData nodeData, @ByRef @Const NodeData nodeData2);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator =="})
    public static native boolean equals(@ByRef @Const Range range, @ByRef @Const Range range2);

    @Namespace("cv")
    public static native void error(int i, @opencv_core.Str String str, String str2, String str3, int i2);

    @Namespace("cv")
    public static native void error(int i, @opencv_core.Str BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"const char*"}) BytePointer bytePointer3, int i2);

    @Namespace("cv::hal")
    public static native void exp(@Const DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, int i);

    @Namespace("cv::hal")
    public static native void exp(@Const FloatBuffer floatBuffer, FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native void exp(@Const DoublePointer doublePointer, DoublePointer doublePointer2, int i);

    @Namespace("cv::hal")
    public static native void exp(@Const FloatPointer floatPointer, FloatPointer floatPointer2, int i);

    @Namespace("cv")
    public static native void exp(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void exp(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void exp(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv::hal")
    public static native void exp(@Const double[] dArr, double[] dArr2, int i);

    @Namespace("cv::hal")
    public static native void exp(@Const float[] fArr, float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native void exp32f(@Const FloatBuffer floatBuffer, FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native void exp32f(@Const FloatPointer floatPointer, FloatPointer floatPointer2, int i);

    @Namespace("cv::hal")
    public static native void exp32f(@Const float[] fArr, float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native void exp64f(@Const DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, int i);

    @Namespace("cv::hal")
    public static native void exp64f(@Const DoublePointer doublePointer, DoublePointer doublePointer2, int i);

    @Namespace("cv::hal")
    public static native void exp64f(@Const double[] dArr, double[] dArr2, int i);

    @Namespace("cv")
    public static native void extractChannel(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native void extractChannel(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native void extractChannel(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    public static native void extractImageCOI(@Const opencv_core.CvArr cvArr, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void extractImageCOI(@Const opencv_core.CvArr cvArr, @ByVal GpuMat gpuMat, int i);

    @Namespace("cv")
    public static native void extractImageCOI(@Const opencv_core.CvArr cvArr, @ByVal Mat mat);

    @Namespace("cv")
    public static native void extractImageCOI(@Const opencv_core.CvArr cvArr, @ByVal Mat mat, int i);

    @Namespace("cv")
    public static native void extractImageCOI(@Const opencv_core.CvArr cvArr, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void extractImageCOI(@Const opencv_core.CvArr cvArr, @ByVal UMat uMat, int i);

    @Namespace("cv")
    public static native float fastAtan2(float f, float f2);

    @Namespace("cv::hal")
    public static native void fastAtan2(@Const FloatBuffer floatBuffer, @Const FloatBuffer floatBuffer2, FloatBuffer floatBuffer3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv::hal")
    public static native void fastAtan2(@Const FloatPointer floatPointer, @Const FloatPointer floatPointer2, FloatPointer floatPointer3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv::hal")
    public static native void fastAtan2(@Const float[] fArr, @Const float[] fArr2, float[] fArr3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv::hal")
    public static native void fastAtan32f(@Const FloatBuffer floatBuffer, @Const FloatBuffer floatBuffer2, FloatBuffer floatBuffer3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv::hal")
    public static native void fastAtan32f(@Const FloatPointer floatPointer, @Const FloatPointer floatPointer2, FloatPointer floatPointer3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv::hal")
    public static native void fastAtan32f(@Const float[] fArr, @Const float[] fArr2, float[] fArr3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv::hal")
    public static native void fastAtan64f(@Const DoubleBuffer doubleBuffer, @Const DoubleBuffer doubleBuffer2, DoubleBuffer doubleBuffer3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv::hal")
    public static native void fastAtan64f(@Const DoublePointer doublePointer, @Const DoublePointer doublePointer2, DoublePointer doublePointer3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv::hal")
    public static native void fastAtan64f(@Const double[] dArr, @Const double[] dArr2, double[] dArr3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void fastFree(Pointer pointer);

    @Namespace("cv")
    public static native Pointer fastMalloc(@Cast({"size_t"}) long j);

    @Namespace("cv::samples")
    @opencv_core.Str
    public static native String findFile(@opencv_core.Str String str);

    @Namespace("cv::samples")
    @opencv_core.Str
    public static native String findFile(@opencv_core.Str String str, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2);

    @Namespace("cv::samples")
    @opencv_core.Str
    public static native BytePointer findFile(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::samples")
    @opencv_core.Str
    public static native BytePointer findFile(@opencv_core.Str BytePointer bytePointer, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2);

    @Namespace("cv::samples")
    @opencv_core.Str
    public static native String findFileOrKeep(@opencv_core.Str String str);

    @Namespace("cv::samples")
    @opencv_core.Str
    public static native String findFileOrKeep(@opencv_core.Str String str, @Cast({"bool"}) boolean z);

    @Namespace("cv::samples")
    @opencv_core.Str
    public static native BytePointer findFileOrKeep(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::samples")
    @opencv_core.Str
    public static native BytePointer findFileOrKeep(@opencv_core.Str BytePointer bytePointer, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void findNonZero(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void findNonZero(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void findNonZero(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv::ocl")
    public static native void finish();

    @Namespace("cv")
    public static native void flip(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native void flip(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native void flip(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    @ByVal
    @Name({"saturate_cast<cv::float16_t>"})
    public static native float16_t float16SaturateCast(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @ByVal
    @Name({"saturate_cast<cv::float16_t>"})
    public static native float16_t float16SaturateCast(double d);

    @Namespace("cv")
    @ByVal
    @Name({"saturate_cast<cv::float16_t>"})
    public static native float16_t float16SaturateCast(float f);

    @Namespace("cv")
    @ByVal
    @Name({"saturate_cast<cv::float16_t>"})
    public static native float16_t float16SaturateCast(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @ByVal
    @Name({"saturate_cast<cv::float16_t>"})
    public static native float16_t float16SaturateCast(@Cast({"int64"}) long j);

    @Namespace("cv")
    @ByVal
    @Name({"saturate_cast<cv::float16_t>"})
    public static native float16_t float16SaturateCast(@ByVal float16_t float16_t2);

    @Namespace("cv")
    @ByVal
    @Name({"saturate_cast<cv::float16_t>"})
    public static native float16_t float16SaturateCast(@Cast({"ushort"}) short s);

    @Namespace("cv")
    @Name({"randu<float>"})
    public static native float floatRand();

    @Namespace("cv")
    @opencv_core.Str
    public static native String format(String str);

    @Namespace("cv")
    @opencv_core.Str
    public static native BytePointer format(@Cast({"const char*"}) BytePointer bytePointer);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native Formatted format(@ByVal GpuMat gpuMat, @Cast({"cv::Formatter::FormatType"}) int i);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native Formatted format(@ByVal Mat mat, @Cast({"cv::Formatter::FormatType"}) int i);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native Formatted format(@ByVal UMat uMat, @Cast({"cv::Formatter::FormatType"}) int i);

    @Namespace("cv")
    public static native void gemm(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, double d, @ByVal GpuMat gpuMat3, double d2, @ByVal GpuMat gpuMat4);

    @Namespace("cv")
    public static native void gemm(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, double d, @ByVal GpuMat gpuMat3, double d2, @ByVal GpuMat gpuMat4, int i);

    @Namespace("cv")
    public static native void gemm(@ByVal Mat mat, @ByVal Mat mat2, double d, @ByVal Mat mat3, double d2, @ByVal Mat mat4);

    @Namespace("cv")
    public static native void gemm(@ByVal Mat mat, @ByVal Mat mat2, double d, @ByVal Mat mat3, double d2, @ByVal Mat mat4, int i);

    @Namespace("cv")
    public static native void gemm(@ByVal UMat uMat, @ByVal UMat uMat2, double d, @ByVal UMat uMat3, double d2, @ByVal UMat uMat4);

    @Namespace("cv")
    public static native void gemm(@ByVal UMat uMat, @ByVal UMat uMat2, double d, @ByVal UMat uMat3, double d2, @ByVal UMat uMat4, int i);

    @Namespace("cv::hal")
    public static native void gemm32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, float f, @Const FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, float f2, FloatBuffer floatBuffer4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, float f, @Const FloatPointer floatPointer3, @Cast({"size_t"}) long j3, float f2, FloatPointer floatPointer4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float f, @Const float[] fArr3, @Cast({"size_t"}) long j3, float f2, float[] fArr4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm32fc(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, float f, @Const FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, float f2, FloatBuffer floatBuffer4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm32fc(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, float f, @Const FloatPointer floatPointer3, @Cast({"size_t"}) long j3, float f2, FloatPointer floatPointer4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm32fc(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float f, @Const float[] fArr3, @Cast({"size_t"}) long j3, float f2, float[] fArr4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, double d, @Const DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, double d2, DoubleBuffer doubleBuffer4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, double d, @Const DoublePointer doublePointer3, @Cast({"size_t"}) long j3, double d2, DoublePointer doublePointer4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double d, @Const double[] dArr3, @Cast({"size_t"}) long j3, double d2, double[] dArr4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm64fc(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, double d, @Const DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, double d2, DoubleBuffer doubleBuffer4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm64fc(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, double d, @Const DoublePointer doublePointer3, @Cast({"size_t"}) long j3, double d2, DoublePointer doublePointer4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv::hal")
    public static native void gemm64fc(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double d, @Const double[] dArr3, @Cast({"size_t"}) long j3, double d2, double[] dArr4, @Cast({"size_t"}) long j4, int i, int i2, int i3, int i4);

    @Namespace("cv")
    @opencv_core.Str
    public static native BytePointer getBuildInformation();

    @Namespace("cv")
    @StdString
    public static native BytePointer getCPUFeaturesLine();

    @Namespace("cv")
    @Cast({"int64"})
    public static native long getCPUTickCount();

    @Namespace("cv::cuda")
    public static native int getCudaEnabledDeviceCount();

    @Namespace("cv::cuda")
    public static native int getDevice();

    @Namespace("cv")
    @Cast({"size_t"})
    public static native long getElemSize(int i);

    @Namespace("cv::instr")
    @Cast({"cv::instr::FLAGS"})
    public static native int getFlags();

    @Namespace("cv")
    @opencv_core.Str
    public static native BytePointer getHardwareFeatureName(int i);

    @Namespace("cv::ipp")
    @opencv_core.Str
    public static native BytePointer getIppErrorLocation();

    @Namespace("cv::ipp")
    @Cast({"unsigned long long"})
    public static native long getIppFeatures();

    @Namespace("cv::ipp")
    public static native int getIppStatus();

    @Namespace("cv::ipp")
    @opencv_core.Str
    public static native BytePointer getIppVersion();

    @Namespace("cv")
    public static native int getNumThreads();

    @Namespace("cv")
    public static native int getNumberOfCPUs();

    @Namespace("cv::ocl")
    public static native MatAllocator getOpenCLAllocator();

    @Namespace("cv::ocl")
    @Cast({"const char*"})
    public static native BytePointer getOpenCLErrorString(int i);

    @Namespace("cv")
    public static native int getOptimalDFTSize(int i);

    @Namespace("cv::ocl")
    public static native void getPlatfomsInfo(@StdVector PlatformInfo platformInfo);

    @Namespace("cv")
    @Cast({"schar*"})
    public static native BytePointer getSeqElem(@Const CvSeq cvSeq, int i);

    @Namespace("cv::utils")
    public static native int getThreadID();

    @Namespace("cv")
    public static native int getThreadNum();

    @Namespace("cv")
    @Cast({"int64"})
    public static native long getTickCount();

    @Namespace("cv")
    public static native double getTickFrequency();

    @Namespace("cv::instr")
    public static native InstrNode getTrace();

    @Namespace("cv")
    public static native int getVersionMajor();

    @Namespace("cv")
    public static native int getVersionMinor();

    @Namespace("cv")
    public static native int getVersionRevision();

    @Namespace("cv")
    @opencv_core.Str
    public static native BytePointer getVersionString();

    @Namespace("cv")
    public static native void glob(@opencv_core.Str String str, @ByRef StringVector stringVector);

    @Namespace("cv")
    public static native void glob(@opencv_core.Str String str, @ByRef StringVector stringVector, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void glob(@opencv_core.Str BytePointer bytePointer, @ByRef StringVector stringVector);

    @Namespace("cv")
    public static native void glob(@opencv_core.Str BytePointer bytePointer, @ByRef StringVector stringVector, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @ByVal
    @Name({"operator >"})
    public static native MatExpr greaterThan(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator >"})
    public static native MatExpr greaterThan(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator >"})
    public static native MatExpr greaterThan(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByVal
    @Name({"operator >="})
    public static native MatExpr greaterThanEquals(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator >="})
    public static native MatExpr greaterThanEquals(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator >="})
    public static native MatExpr greaterThanEquals(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv::ocl")
    @Cast({"bool"})
    public static native boolean haveAmdBlas();

    @Namespace("cv::ocl")
    @Cast({"bool"})
    public static native boolean haveAmdFft();

    @Namespace("cv::ocl")
    @Cast({"bool"})
    public static native boolean haveOpenCL();

    @Namespace("cv::ocl")
    @Cast({"bool"})
    public static native boolean haveSVM();

    @Namespace("cv")
    public static native void hconcat(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void hconcat(@ByVal GpuMatVector gpuMatVector, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void hconcat(@ByVal GpuMatVector gpuMatVector, @ByVal Mat mat);

    @Namespace("cv")
    public static native void hconcat(@ByVal GpuMatVector gpuMatVector, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void hconcat(@Const Mat mat, @Cast({"size_t"}) long j, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void hconcat(@Const Mat mat, @Cast({"size_t"}) long j, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void hconcat(@Const Mat mat, @Cast({"size_t"}) long j, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void hconcat(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void hconcat(@ByVal MatVector matVector, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void hconcat(@ByVal MatVector matVector, @ByVal Mat mat);

    @Namespace("cv")
    public static native void hconcat(@ByVal MatVector matVector, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void hconcat(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void hconcat(@ByVal UMatVector uMatVector, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void hconcat(@ByVal UMatVector uMatVector, @ByVal Mat mat);

    @Namespace("cv")
    public static native void hconcat(@ByVal UMatVector uMatVector, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void idct(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void idct(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native void idct(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void idct(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native void idct(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native void idct(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    public static native void idft(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void idft(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, int i2);

    @Namespace("cv")
    public static native void idft(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void idft(@ByVal Mat mat, @ByVal Mat mat2, int i, int i2);

    @Namespace("cv")
    public static native void idft(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native void idft(@ByVal UMat uMat, @ByVal UMat uMat2, int i, int i2);

    @Namespace("cv")
    public static native void inRange(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4);

    @Namespace("cv")
    public static native void inRange(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4);

    @Namespace("cv")
    public static native void inRange(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4);

    @Namespace("cv")
    public static native void insertChannel(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native void insertChannel(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native void insertChannel(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    public static native void insertImageCOI(@ByVal GpuMat gpuMat, opencv_core.CvArr cvArr);

    @Namespace("cv")
    public static native void insertImageCOI(@ByVal GpuMat gpuMat, opencv_core.CvArr cvArr, int i);

    @Namespace("cv")
    public static native void insertImageCOI(@ByVal Mat mat, opencv_core.CvArr cvArr);

    @Namespace("cv")
    public static native void insertImageCOI(@ByVal Mat mat, opencv_core.CvArr cvArr, int i);

    @Namespace("cv")
    public static native void insertImageCOI(@ByVal UMat uMat, opencv_core.CvArr cvArr);

    @Namespace("cv")
    public static native void insertImageCOI(@ByVal UMat uMat, opencv_core.CvArr cvArr, int i);

    @Namespace("cv")
    @Cast({"int64"})
    @Name({"saturate_cast<int64>"})
    public static native long int64SaturateCast(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @Cast({"int64"})
    @Name({"saturate_cast<int64>"})
    public static native long int64SaturateCast(double d);

    @Namespace("cv")
    @Cast({"int64"})
    @Name({"saturate_cast<int64>"})
    public static native long int64SaturateCast(float f);

    @Namespace("cv")
    @Cast({"int64"})
    @Name({"saturate_cast<int64>"})
    public static native long int64SaturateCast(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @Cast({"int64"})
    @Name({"saturate_cast<int64>"})
    public static native long int64SaturateCast(@Cast({"int64"}) long j);

    @Namespace("cv")
    @Cast({"int64"})
    @Name({"saturate_cast<int64>"})
    public static native long int64SaturateCast(@ByVal float16_t float16_t2);

    @Namespace("cv")
    @Cast({"int64"})
    @Name({"saturate_cast<int64>"})
    public static native long int64SaturateCast(@Cast({"ushort"}) short s);

    @Namespace("cv")
    @Name({"randu<int>"})
    public static native int intRand();

    @Namespace("cv")
    @Name({"saturate_cast<int>"})
    public static native int intSaturate(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @Name({"saturate_cast<int>"})
    public static native int intSaturate(double d);

    @Namespace("cv")
    @Name({"saturate_cast<int>"})
    public static native int intSaturate(float f);

    @Namespace("cv")
    @Name({"saturate_cast<int>"})
    public static native int intSaturate(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @Name({"saturate_cast<int>"})
    public static native int intSaturate(@Cast({"int64"}) long j);

    @Namespace("cv")
    @Name({"saturate_cast<int>"})
    public static native int intSaturate(@ByVal float16_t float16_t2);

    @Namespace("cv")
    @Name({"saturate_cast<int>"})
    public static native int intSaturate(@Cast({"ushort"}) short s);

    @Namespace("cv::hal")
    public static native void invSqrt(@Const DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt(@Const FloatBuffer floatBuffer, FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt(@Const DoublePointer doublePointer, DoublePointer doublePointer2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt(@Const FloatPointer floatPointer, FloatPointer floatPointer2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt(@Const double[] dArr, double[] dArr2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt(@Const float[] fArr, float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt32f(@Const FloatBuffer floatBuffer, FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt32f(@Const FloatPointer floatPointer, FloatPointer floatPointer2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt32f(@Const float[] fArr, float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt64f(@Const DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt64f(@Const DoublePointer doublePointer, DoublePointer doublePointer2, int i);

    @Namespace("cv::hal")
    public static native void invSqrt64f(@Const double[] dArr, double[] dArr2, int i);

    @Namespace("cv")
    public static native double invert(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native double invert(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native double invert(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native double invert(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native double invert(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native double invert(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv::ocl")
    @opencv_core.Str
    public static native String kernelToStr(@ByVal GpuMat gpuMat, int i, String str);

    @Namespace("cv::ocl")
    @opencv_core.Str
    public static native String kernelToStr(@ByVal Mat mat, int i, String str);

    @Namespace("cv::ocl")
    @opencv_core.Str
    public static native String kernelToStr(@ByVal UMat uMat);

    @Namespace("cv::ocl")
    @opencv_core.Str
    public static native String kernelToStr(@ByVal UMat uMat, int i, String str);

    @Namespace("cv::ocl")
    @opencv_core.Str
    public static native BytePointer kernelToStr(@ByVal GpuMat gpuMat);

    @Namespace("cv::ocl")
    @opencv_core.Str
    public static native BytePointer kernelToStr(@ByVal GpuMat gpuMat, int i, @Cast({"const char*"}) BytePointer bytePointer);

    @Namespace("cv::ocl")
    @opencv_core.Str
    public static native BytePointer kernelToStr(@ByVal Mat mat);

    @Namespace("cv::ocl")
    @opencv_core.Str
    public static native BytePointer kernelToStr(@ByVal Mat mat, int i, @Cast({"const char*"}) BytePointer bytePointer);

    @Namespace("cv::ocl")
    @opencv_core.Str
    public static native BytePointer kernelToStr(@ByVal UMat uMat, int i, @Cast({"const char*"}) BytePointer bytePointer);

    @Namespace("cv")
    public static native double kmeans(@ByVal GpuMat gpuMat, int i, @ByVal GpuMat gpuMat2, @ByVal TermCriteria termCriteria, int i2, int i3);

    @Namespace("cv")
    public static native double kmeans(@ByVal GpuMat gpuMat, int i, @ByVal GpuMat gpuMat2, @ByVal TermCriteria termCriteria, int i2, int i3, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") GpuMat gpuMat3);

    @Namespace("cv")
    public static native double kmeans(@ByVal Mat mat, int i, @ByVal Mat mat2, @ByVal TermCriteria termCriteria, int i2, int i3);

    @Namespace("cv")
    public static native double kmeans(@ByVal Mat mat, int i, @ByVal Mat mat2, @ByVal TermCriteria termCriteria, int i2, int i3, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") Mat mat3);

    @Namespace("cv")
    public static native double kmeans(@ByVal UMat uMat, int i, @ByVal UMat uMat2, @ByVal TermCriteria termCriteria, int i2, int i3);

    @Namespace("cv")
    public static native double kmeans(@ByVal UMat uMat, int i, @ByVal UMat uMat2, @ByVal TermCriteria termCriteria, int i2, int i3, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") UMat uMat3);

    @Namespace("cv")
    @ByVal
    @Name({"operator <"})
    public static native MatExpr lessThan(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator <"})
    public static native MatExpr lessThan(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator <"})
    public static native MatExpr lessThan(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator <"})
    public static native boolean lessThan(@ByRef @Const FileNodeIterator fileNodeIterator, @ByRef @Const FileNodeIterator fileNodeIterator2);

    @Namespace("cv")
    @ByVal
    @Name({"operator <="})
    public static native MatExpr lessThanEquals(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator <="})
    public static native MatExpr lessThanEquals(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator <="})
    public static native MatExpr lessThanEquals(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv::hal")
    public static native void log(@Const DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, int i);

    @Namespace("cv::hal")
    public static native void log(@Const FloatBuffer floatBuffer, FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native void log(@Const DoublePointer doublePointer, DoublePointer doublePointer2, int i);

    @Namespace("cv::hal")
    public static native void log(@Const FloatPointer floatPointer, FloatPointer floatPointer2, int i);

    @Namespace("cv")
    public static native void log(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void log(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void log(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv::hal")
    public static native void log(@Const double[] dArr, double[] dArr2, int i);

    @Namespace("cv::hal")
    public static native void log(@Const float[] fArr, float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native void log32f(@Const FloatBuffer floatBuffer, FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native void log32f(@Const FloatPointer floatPointer, FloatPointer floatPointer2, int i);

    @Namespace("cv::hal")
    public static native void log32f(@Const float[] fArr, float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native void log64f(@Const DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, int i);

    @Namespace("cv::hal")
    public static native void log64f(@Const DoublePointer doublePointer, DoublePointer doublePointer2, int i);

    @Namespace("cv::hal")
    public static native void log64f(@Const double[] dArr, double[] dArr2, int i);

    @Namespace("cv::hal")
    public static native void magnitude(@Const DoubleBuffer doubleBuffer, @Const DoubleBuffer doubleBuffer2, DoubleBuffer doubleBuffer3, int i);

    @Namespace("cv::hal")
    public static native void magnitude(@Const FloatBuffer floatBuffer, @Const FloatBuffer floatBuffer2, FloatBuffer floatBuffer3, int i);

    @Namespace("cv::hal")
    public static native void magnitude(@Const DoublePointer doublePointer, @Const DoublePointer doublePointer2, DoublePointer doublePointer3, int i);

    @Namespace("cv::hal")
    public static native void magnitude(@Const FloatPointer floatPointer, @Const FloatPointer floatPointer2, FloatPointer floatPointer3, int i);

    @Namespace("cv")
    public static native void magnitude(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void magnitude(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void magnitude(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv::hal")
    public static native void magnitude(@Const double[] dArr, @Const double[] dArr2, double[] dArr3, int i);

    @Namespace("cv::hal")
    public static native void magnitude(@Const float[] fArr, @Const float[] fArr2, float[] fArr3, int i);

    @Namespace("cv::hal")
    public static native void magnitude32f(@Const FloatBuffer floatBuffer, @Const FloatBuffer floatBuffer2, FloatBuffer floatBuffer3, int i);

    @Namespace("cv::hal")
    public static native void magnitude32f(@Const FloatPointer floatPointer, @Const FloatPointer floatPointer2, FloatPointer floatPointer3, int i);

    @Namespace("cv::hal")
    public static native void magnitude32f(@Const float[] fArr, @Const float[] fArr2, float[] fArr3, int i);

    @Namespace("cv::hal")
    public static native void magnitude64f(@Const DoubleBuffer doubleBuffer, @Const DoubleBuffer doubleBuffer2, DoubleBuffer doubleBuffer3, int i);

    @Namespace("cv::hal")
    public static native void magnitude64f(@Const DoublePointer doublePointer, @Const DoublePointer doublePointer2, DoublePointer doublePointer3, int i);

    @Namespace("cv::hal")
    public static native void magnitude64f(@Const double[] dArr, @Const double[] dArr2, double[] dArr3, int i);

    @Namespace("cv")
    @ByVal
    public static native MatExpr max(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    public static native MatExpr max(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    public static native MatExpr max(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    public static native void max(@ByRef @Const Mat mat, @ByRef @Const Mat mat2, @ByRef Mat mat3);

    @Namespace("cv")
    public static native void max(@ByRef @Const UMat uMat, @ByRef @Const UMat uMat2, @ByRef UMat uMat3);

    @Namespace("cv::hal")
    public static native void max16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, IntBuffer intBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, IntPointer intPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, int[] iArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void max8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    @ByVal
    public static native Scalar mean(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    @ByVal
    public static native Scalar mean(@ByVal GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

    @Namespace("cv")
    @ByVal
    public static native Scalar mean(@ByVal Mat mat);

    @Namespace("cv")
    @ByVal
    public static native Scalar mean(@ByVal Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

    @Namespace("cv")
    @ByVal
    public static native Scalar mean(@ByVal UMat uMat);

    @Namespace("cv")
    @ByVal
    public static native Scalar mean(@ByVal UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

    @Namespace("cv")
    public static native void meanStdDev(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void meanStdDev(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4);

    @Namespace("cv")
    public static native void meanStdDev(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void meanStdDev(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4);

    @Namespace("cv")
    public static native void meanStdDev(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void meanStdDev(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4);

    @Namespace("cv::ocl")
    @Cast({"const char*"})
    public static native BytePointer memopTypeToStr(int i);

    @Namespace("cv")
    public static native void merge(@ByVal GpuMatVector gpuMatVector, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void merge(@ByVal GpuMatVector gpuMatVector, @ByVal Mat mat);

    @Namespace("cv")
    public static native void merge(@ByVal GpuMatVector gpuMatVector, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void merge(@Const Mat mat, @Cast({"size_t"}) long j, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void merge(@Const Mat mat, @Cast({"size_t"}) long j, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void merge(@Const Mat mat, @Cast({"size_t"}) long j, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void merge(@ByVal MatVector matVector, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void merge(@ByVal MatVector matVector, @ByVal Mat mat);

    @Namespace("cv")
    public static native void merge(@ByVal MatVector matVector, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void merge(@ByVal UMatVector uMatVector, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void merge(@ByVal UMatVector uMatVector, @ByVal Mat mat);

    @Namespace("cv")
    public static native void merge(@ByVal UMatVector uMatVector, @ByVal UMat uMat);

    @Namespace("cv::hal")
    public static native void merge16u(@ByPtrPtr @Cast({"const ushort**"}) ShortBuffer shortBuffer, @Cast({"ushort*"}) ShortBuffer shortBuffer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge16u(@Cast({"const ushort**"}) PointerPointer pointerPointer, @Cast({"ushort*"}) ShortPointer shortPointer, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge16u(@ByPtrPtr @Cast({"const ushort**"}) ShortPointer shortPointer, @Cast({"ushort*"}) ShortPointer shortPointer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge16u(@ByPtrPtr @Cast({"const ushort**"}) short[] sArr, @Cast({"ushort*"}) short[] sArr2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge32s(@ByPtrPtr @Const IntBuffer intBuffer, IntBuffer intBuffer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge32s(@ByPtrPtr @Const IntPointer intPointer, IntPointer intPointer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge32s(@Cast({"const int**"}) PointerPointer pointerPointer, IntPointer intPointer, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge32s(@ByPtrPtr @Const int[] iArr, int[] iArr2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge64s(@ByPtrPtr @Cast({"const int64**"}) LongBuffer longBuffer, @Cast({"int64*"}) LongBuffer longBuffer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge64s(@ByPtrPtr @Cast({"const int64**"}) LongPointer longPointer, @Cast({"int64*"}) LongPointer longPointer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge64s(@Cast({"const int64**"}) PointerPointer pointerPointer, @Cast({"int64*"}) LongPointer longPointer, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge64s(@ByPtrPtr @Cast({"const int64**"}) long[] jArr, @Cast({"int64*"}) long[] jArr2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge8u(@ByPtrPtr @Cast({"const uchar**"}) ByteBuffer byteBuffer, @Cast({"uchar*"}) ByteBuffer byteBuffer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge8u(@ByPtrPtr @Cast({"const uchar**"}) BytePointer bytePointer, @Cast({"uchar*"}) BytePointer bytePointer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge8u(@Cast({"const uchar**"}) PointerPointer pointerPointer, @Cast({"uchar*"}) BytePointer bytePointer, int i, int i2);

    @Namespace("cv::hal")
    public static native void merge8u(@ByPtrPtr @Cast({"const uchar**"}) byte[] bArr, @Cast({"uchar*"}) byte[] bArr2, int i, int i2);

    @Namespace("cv")
    @ByVal
    public static native MatExpr min(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    public static native MatExpr min(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    public static native MatExpr min(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    public static native void min(@ByRef @Const Mat mat, @ByRef @Const Mat mat2, @ByRef Mat mat3);

    @Namespace("cv")
    public static native void min(@ByRef @Const UMat uMat, @ByRef @Const UMat uMat2, @ByRef UMat uMat3);

    @Namespace("cv::hal")
    public static native void min16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, IntBuffer intBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, IntPointer intPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, int[] iArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void min8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal GpuMat gpuMat, DoubleBuffer doubleBuffer);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal GpuMat gpuMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, IntBuffer intBuffer, IntBuffer intBuffer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal GpuMat gpuMat, DoublePointer doublePointer);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal GpuMat gpuMat, DoublePointer doublePointer, DoublePointer doublePointer2, IntPointer intPointer, IntPointer intPointer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal GpuMat gpuMat, double[] dArr);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal GpuMat gpuMat, double[] dArr, double[] dArr2, int[] iArr, int[] iArr2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal Mat mat, DoubleBuffer doubleBuffer);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal Mat mat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, IntBuffer intBuffer, IntBuffer intBuffer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal Mat mat, DoublePointer doublePointer);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal Mat mat, DoublePointer doublePointer, DoublePointer doublePointer2, IntPointer intPointer, IntPointer intPointer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal Mat mat, double[] dArr);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal Mat mat, double[] dArr, double[] dArr2, int[] iArr, int[] iArr2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal UMat uMat, DoubleBuffer doubleBuffer);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal UMat uMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, IntBuffer intBuffer, IntBuffer intBuffer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal UMat uMat, DoublePointer doublePointer);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal UMat uMat, DoublePointer doublePointer, DoublePointer doublePointer2, IntPointer intPointer, IntPointer intPointer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal UMat uMat, double[] dArr);

    @Namespace("cv")
    public static native void minMaxIdx(@ByVal UMat uMat, double[] dArr, double[] dArr2, int[] iArr, int[] iArr2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal GpuMat gpuMat, DoubleBuffer doubleBuffer);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal GpuMat gpuMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, Point point, Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal GpuMat gpuMat, DoublePointer doublePointer);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal GpuMat gpuMat, DoublePointer doublePointer, DoublePointer doublePointer2, Point point, Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal GpuMat gpuMat, double[] dArr);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal GpuMat gpuMat, double[] dArr, double[] dArr2, Point point, Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal Mat mat, DoubleBuffer doubleBuffer);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal Mat mat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, Point point, Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal Mat mat, DoublePointer doublePointer);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal Mat mat, DoublePointer doublePointer, DoublePointer doublePointer2, Point point, Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal Mat mat, double[] dArr);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal Mat mat, double[] dArr, double[] dArr2, Point point, Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByRef @Const SparseMat sparseMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByRef @Const SparseMat sparseMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, IntBuffer intBuffer, IntBuffer intBuffer2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByRef @Const SparseMat sparseMat, DoublePointer doublePointer, DoublePointer doublePointer2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByRef @Const SparseMat sparseMat, DoublePointer doublePointer, DoublePointer doublePointer2, IntPointer intPointer, IntPointer intPointer2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByRef @Const SparseMat sparseMat, double[] dArr, double[] dArr2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByRef @Const SparseMat sparseMat, double[] dArr, double[] dArr2, int[] iArr, int[] iArr2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal UMat uMat, DoubleBuffer doubleBuffer);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal UMat uMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, Point point, Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal UMat uMat, DoublePointer doublePointer);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal UMat uMat, DoublePointer doublePointer, DoublePointer doublePointer2, Point point, Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal UMat uMat, double[] dArr);

    @Namespace("cv")
    public static native void minMaxLoc(@ByVal UMat uMat, double[] dArr, double[] dArr2, Point point, Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

    @Namespace("cv")
    public static native void mixChannels(@ByVal GpuMatVector gpuMatVector, @ByVal GpuMatVector gpuMatVector2, @StdVector int[] iArr);

    @Namespace("cv")
    public static native void mixChannels(@ByVal GpuMatVector gpuMatVector, @ByVal GpuMatVector gpuMatVector2, @Const int[] iArr, @Cast({"size_t"}) long j);

    @Namespace("cv")
    public static native void mixChannels(@Const Mat mat, @Cast({"size_t"}) long j, Mat mat2, @Cast({"size_t"}) long j2, @Const IntBuffer intBuffer, @Cast({"size_t"}) long j3);

    @Namespace("cv")
    public static native void mixChannels(@Const Mat mat, @Cast({"size_t"}) long j, Mat mat2, @Cast({"size_t"}) long j2, @Const IntPointer intPointer, @Cast({"size_t"}) long j3);

    @Namespace("cv")
    public static native void mixChannels(@Const Mat mat, @Cast({"size_t"}) long j, Mat mat2, @Cast({"size_t"}) long j2, @Const int[] iArr, @Cast({"size_t"}) long j3);

    @Namespace("cv")
    public static native void mixChannels(@ByVal MatVector matVector, @ByVal MatVector matVector2, @StdVector IntPointer intPointer);

    @Namespace("cv")
    public static native void mixChannels(@ByVal MatVector matVector, @ByVal MatVector matVector2, @Const IntPointer intPointer, @Cast({"size_t"}) long j);

    @Namespace("cv")
    public static native void mixChannels(@ByVal UMatVector uMatVector, @ByVal UMatVector uMatVector2, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    public static native void mixChannels(@ByVal UMatVector uMatVector, @ByVal UMatVector uMatVector2, @Const IntBuffer intBuffer, @Cast({"size_t"}) long j);

    @Namespace("cv::hal")
    public static native void mul16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, IntBuffer intBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, IntPointer intPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, int[] iArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void mul8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    public static native void mulSpectrums(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, int i);

    @Namespace("cv")
    public static native void mulSpectrums(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void mulSpectrums(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, int i);

    @Namespace("cv")
    public static native void mulSpectrums(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void mulSpectrums(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, int i);

    @Namespace("cv")
    public static native void mulSpectrums(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void mulTransposed(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void mulTransposed(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat3, double d, int i);

    @Namespace("cv")
    public static native void mulTransposed(@ByVal Mat mat, @ByVal Mat mat2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void mulTransposed(@ByVal Mat mat, @ByVal Mat mat2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat3, double d, int i);

    @Namespace("cv")
    public static native void mulTransposed(@ByVal UMat uMat, @ByVal UMat uMat2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void mulTransposed(@ByVal UMat uMat, @ByVal UMat uMat2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat3, double d, int i);

    @Namespace("cv")
    @ByVal
    @Name({"operator *"})
    public static native MatExpr multiply(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator *"})
    public static native MatExpr multiply(double d, @ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @ByVal
    @Name({"operator *"})
    public static native MatExpr multiply(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator *"})
    public static native MatExpr multiply(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByVal
    @Name({"operator *"})
    public static native MatExpr multiply(@ByRef @Const Mat mat, @ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @ByVal
    @Name({"operator *"})
    public static native MatExpr multiply(@ByRef @Const MatExpr matExpr, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator *"})
    public static native MatExpr multiply(@ByRef @Const MatExpr matExpr, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator *"})
    public static native MatExpr multiply(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2);

    @Namespace("cv")
    public static native void multiply(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void multiply(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, double d, int i);

    @Namespace("cv")
    public static native void multiply(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void multiply(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, double d, int i);

    @Namespace("cv")
    public static native void multiply(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void multiply(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, double d, int i);

    @Namespace("cv")
    @ByRef
    @Name({"operator *="})
    public static native Mat multiplyPut(@ByRef Mat mat, double d);

    @Namespace("cv")
    @ByRef
    @Name({"operator *="})
    public static native Mat multiplyPut(@ByRef Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    public static native double norm(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native double norm(@ByVal GpuMat gpuMat, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

    @Namespace("cv")
    public static native double norm(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native double norm(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat3);

    @Namespace("cv")
    public static native double norm(@ByVal Mat mat);

    @Namespace("cv")
    public static native double norm(@ByVal Mat mat, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

    @Namespace("cv")
    public static native double norm(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native double norm(@ByVal Mat mat, @ByVal Mat mat2, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat3);

    @Namespace("cv")
    public static native double norm(@ByRef @Const SparseMat sparseMat, int i);

    @Namespace("cv")
    public static native double norm(@ByVal UMat uMat);

    @Namespace("cv")
    public static native double norm(@ByVal UMat uMat, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

    @Namespace("cv")
    public static native double norm(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native double norm(@ByVal UMat uMat, @ByVal UMat uMat2, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat3);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) ByteBuffer byteBuffer, int i);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) ByteBuffer byteBuffer, int i, int i2);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, int i);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, int i, int i2);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) BytePointer bytePointer, int i);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) BytePointer bytePointer, int i, int i2);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"const uchar*"}) BytePointer bytePointer2, int i);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"const uchar*"}) BytePointer bytePointer2, int i, int i2);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) byte[] bArr, int i);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) byte[] bArr, int i, int i2);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) byte[] bArr, @Cast({"const uchar*"}) byte[] bArr2, int i);

    @Namespace("cv::hal")
    public static native int normHamming(@Cast({"const uchar*"}) byte[] bArr, @Cast({"const uchar*"}) byte[] bArr2, int i, int i2);

    @Namespace("cv::hal")
    public static native float normL1_(@Const FloatBuffer floatBuffer, @Const FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native float normL1_(@Const FloatPointer floatPointer, @Const FloatPointer floatPointer2, int i);

    @Namespace("cv::hal")
    public static native float normL1_(@Const float[] fArr, @Const float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native int normL1_(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, int i);

    @Namespace("cv::hal")
    public static native int normL1_(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"const uchar*"}) BytePointer bytePointer2, int i);

    @Namespace("cv::hal")
    public static native int normL1_(@Cast({"const uchar*"}) byte[] bArr, @Cast({"const uchar*"}) byte[] bArr2, int i);

    @Namespace("cv::hal")
    public static native float normL2Sqr_(@Const FloatBuffer floatBuffer, @Const FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native float normL2Sqr_(@Const FloatPointer floatPointer, @Const FloatPointer floatPointer2, int i);

    @Namespace("cv::hal")
    public static native float normL2Sqr_(@Const float[] fArr, @Const float[] fArr2, int i);

    @Namespace("cv")
    public static native void normalize(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void normalize(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, double d, double d2, int i, int i2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat3);

    @Namespace("cv")
    public static native void normalize(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void normalize(@ByVal Mat mat, @ByVal Mat mat2, double d, double d2, int i, int i2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat3);

    @Namespace("cv")
    public static native void normalize(@ByRef @Const SparseMat sparseMat, @ByRef SparseMat sparseMat2, double d, int i);

    @Namespace("cv")
    public static native void normalize(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native void normalize(@ByVal UMat uMat, @ByVal UMat uMat2, double d, double d2, int i, int i2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat3);

    @Namespace("cv")
    @ByVal
    @Name({"operator ~"})
    public static native MatExpr not(@ByRef @Const Mat mat);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator !"})
    public static native boolean not(@Cast({"const cv::AccessFlag"}) int i);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator !"})
    public static native boolean not(@ByRef @Const Range range);

    @Namespace("cv::hal")
    public static native void not8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void not8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void not8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    @ByVal
    @Name({"operator !="})
    public static native MatExpr notEquals(double d, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator !="})
    public static native MatExpr notEquals(@ByRef @Const Mat mat, double d);

    @Namespace("cv")
    @ByVal
    @Name({"operator !="})
    public static native MatExpr notEquals(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator !="})
    public static native boolean notEquals(@Cast({"const cv::AccessFlag"}) int i, int i2);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator !="})
    public static native boolean notEquals(@ByRef @Const FileNodeIterator fileNodeIterator, @ByRef @Const FileNodeIterator fileNodeIterator2);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator !="})
    public static native boolean notEquals(@ByRef @Const Range range, @ByRef @Const Range range2);

    @Namespace("cv")
    @Cast({"cv::AccessFlag"})
    @Name({"operator |"})
    public static native int or(@Cast({"const cv::AccessFlag"}) int i, @Cast({"const cv::AccessFlag"}) int i2);

    @Namespace("cv")
    @ByVal
    @Name({"operator |"})
    public static native MatExpr or(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByVal
    @Name({"operator |"})
    public static native MatExpr or(@ByRef @Const Mat mat, @ByRef @Const Scalar scalar);

    @Namespace("cv")
    @ByVal
    @Name({"operator |"})
    public static native MatExpr or(@ByRef @Const Scalar scalar, @ByRef @Const Mat mat);

    @Namespace("cv::hal")
    public static native void or8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void or8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void or8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @ByRef
    @Cast({"cv::AccessFlag*"})
    @Name({"operator |="})
    @Namespace("cv")
    public static native IntBuffer orPut(@ByRef @Cast({"cv::AccessFlag*"}) IntBuffer intBuffer, @Cast({"const cv::AccessFlag"}) int i);

    @ByRef
    @Cast({"cv::AccessFlag*"})
    @Name({"operator |="})
    @Namespace("cv")
    public static native IntPointer orPut(@ByRef @Cast({"cv::AccessFlag*"}) IntPointer intPointer, @Cast({"const cv::AccessFlag"}) int i);

    @Namespace("cv")
    @ByRef
    @Name({"operator |="})
    public static native Mat orPut(@ByRef Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByRef
    @Name({"operator |="})
    public static native Mat orPut(@ByRef Mat mat, @ByRef @Const Scalar scalar);

    @ByRef
    @Cast({"cv::AccessFlag*"})
    @Name({"operator |="})
    @Namespace("cv")
    public static native int[] orPut(@ByRef @Cast({"cv::AccessFlag*"}) int[] iArr, @Cast({"const cv::AccessFlag"}) int i);

    @Namespace("cv")
    public static native void parallel_for_(@ByRef @Const Range range, @ByRef @Const ParallelLoopBody parallelLoopBody);

    @Namespace("cv")
    public static native void parallel_for_(@ByRef @Const Range range, @ByRef @Const ParallelLoopBody parallelLoopBody, double d);

    @Namespace("cv")
    public static native void parallel_for_(@ByRef @Const Range range, @ByVal opencv_core.Functor functor);

    @Namespace("cv")
    public static native void parallel_for_(@ByRef @Const Range range, @ByVal opencv_core.Functor functor, double d);

    @Namespace("cv")
    public static native void patchNaNs(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void patchNaNs(@ByVal GpuMat gpuMat, double d);

    @Namespace("cv")
    public static native void patchNaNs(@ByVal Mat mat);

    @Namespace("cv")
    public static native void patchNaNs(@ByVal Mat mat, double d);

    @Namespace("cv")
    public static native void patchNaNs(@ByVal UMat uMat);

    @Namespace("cv")
    public static native void patchNaNs(@ByVal UMat uMat, double d);

    @Namespace("cv")
    public static native void perspectiveTransform(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void perspectiveTransform(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void perspectiveTransform(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void phase(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void phase(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void phase(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void phase(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void phase(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void phase(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void polarToCart(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4);

    @Namespace("cv")
    public static native void polarToCart(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void polarToCart(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4);

    @Namespace("cv")
    public static native void polarToCart(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void polarToCart(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4);

    @Namespace("cv")
    public static native void polarToCart(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void pow(@ByVal GpuMat gpuMat, double d, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void pow(@ByVal Mat mat, double d, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void pow(@ByVal UMat uMat, double d, @ByVal UMat uMat2);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidth(@ByVal GpuMat gpuMat);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidth(@ByVal GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidth(@ByVal Mat mat);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidth(@ByVal Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidth(@ByVal UMat uMat);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidth(@ByVal UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat9, @Cast({"cv::ocl::OclVectorStrategy"}) int i);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidthMax(@ByVal GpuMat gpuMat);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidthMax(@ByVal GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat9);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidthMax(@ByVal Mat mat);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidthMax(@ByVal Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat9);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidthMax(@ByVal UMat uMat);

    @Namespace("cv::ocl")
    public static native int predictOptimalVectorWidthMax(@ByVal UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat6, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat7, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat8, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat9);

    @Namespace("cv")
    public static native int print(@opencv_core.Ptr Formatted formatted);

    @Namespace("cv")
    public static native int print(@opencv_core.Ptr Formatted formatted, @Cast({"FILE*"}) Pointer pointer);

    @Namespace("cv")
    public static native int print(@ByRef @Const Mat mat);

    @Namespace("cv")
    public static native int print(@ByRef @Const Mat mat, @Cast({"FILE*"}) Pointer pointer);

    @Namespace("cv")
    public static native int print(@ByRef @Const UMat uMat);

    @Namespace("cv")
    public static native int print(@ByRef @Const UMat uMat, @Cast({"FILE*"}) Pointer pointer);

    @Namespace("cv::cuda")
    public static native void printCudaDeviceInfo(int i);

    @Namespace("cv::cuda")
    public static native void printShortCudaDeviceInfo(int i);

    @Namespace("cv")
    public static native void randShuffle(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void randShuffle(@ByVal GpuMat gpuMat, double d, RNG rng);

    @Namespace("cv")
    public static native void randShuffle(@ByVal Mat mat);

    @Namespace("cv")
    public static native void randShuffle(@ByVal Mat mat, double d, RNG rng);

    @Namespace("cv")
    public static native void randShuffle(@ByVal UMat uMat);

    @Namespace("cv")
    public static native void randShuffle(@ByVal UMat uMat, double d, RNG rng);

    @Namespace("cv")
    public static native void randn(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void randn(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void randn(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void randu(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void randu(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void randu(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @Cast({"uchar*"}) ByteBuffer byteBuffer, @Cast({"uchar"}) byte b);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef DoubleBuffer doubleBuffer, double d);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef FloatBuffer floatBuffer, float f);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef IntBuffer intBuffer, int i);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @Cast({"ushort*"}) ShortBuffer shortBuffer, @Cast({"ushort"}) short s);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @Cast({"bool*"}) BoolPointer boolPointer, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @Cast({"uchar*"}) BytePointer bytePointer, @Cast({"uchar"}) byte b);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @StdString BytePointer bytePointer, @StdString String str);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @StdString BytePointer bytePointer, @StdString BytePointer bytePointer2);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef DoublePointer doublePointer, double d);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef FloatPointer floatPointer, float f);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef IntPointer intPointer, int i);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @Cast({"ushort*"}) ShortPointer shortPointer, @Cast({"ushort"}) short s);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef DMatch dMatch, @ByRef @Const DMatch dMatch2);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef DMatchVector dMatchVector);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef DMatchVector dMatchVector, @ByRef @Const DMatchVector dMatchVector2);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef KeyPoint keyPoint, @ByRef @Const KeyPoint keyPoint2);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef KeyPointVector keyPointVector);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef KeyPointVector keyPointVector, @ByRef @Const KeyPointVector keyPointVector2);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef Mat mat);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef Mat mat, @ByRef(nullValue = "cv::Mat()") @Const Mat mat2);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef Range range, @ByRef @Const Range range2);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef SparseMat sparseMat);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef SparseMat sparseMat, @ByRef(nullValue = "cv::SparseMat()") @Const SparseMat sparseMat2);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @Cast({"uchar*"}) byte[] bArr, @Cast({"uchar"}) byte b);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef double[] dArr, double d);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef float[] fArr, float f);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef int[] iArr, int i);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @Cast({"ushort*"}) short[] sArr, @Cast({"ushort"}) short s);

    @Namespace("cv")
    public static native void read(@ByRef @Const FileNode fileNode, @ByRef @Cast({"bool*"}) boolean[] zArr, @Cast({"bool"}) boolean z);

    @Namespace("cv::hal")
    public static native void recip16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, IntBuffer intBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, IntPointer intPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, int[] iArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void recip8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    public static native ErrorCallback redirectError(ErrorCallback errorCallback);

    @Namespace("cv")
    public static native ErrorCallback redirectError(ErrorCallback errorCallback, Pointer pointer, @ByPtrPtr @Cast({"void**"}) Pointer pointer2);

    @Namespace("cv")
    public static native ErrorCallback redirectError(ErrorCallback errorCallback, Pointer pointer, @Cast({"void**"}) PointerPointer pointerPointer);

    @Namespace("cv")
    public static native void reduce(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, int i2);

    @Namespace("cv")
    public static native void reduce(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, int i2, int i3);

    @Namespace("cv")
    public static native void reduce(@ByVal Mat mat, @ByVal Mat mat2, int i, int i2);

    @Namespace("cv")
    public static native void reduce(@ByVal Mat mat, @ByVal Mat mat2, int i, int i2, int i3);

    @Namespace("cv")
    public static native void reduce(@ByVal UMat uMat, @ByVal UMat uMat2, int i, int i2);

    @Namespace("cv")
    public static native void reduce(@ByVal UMat uMat, @ByVal UMat uMat2, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void registerPageLocked(@ByRef Mat mat);

    @Namespace("cv")
    @ByVal
    public static native Mat repeat(@ByRef @Const Mat mat, int i, int i2);

    @Namespace("cv")
    public static native void repeat(@ByVal GpuMat gpuMat, int i, int i2, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void repeat(@ByVal Mat mat, int i, int i2, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void repeat(@ByVal UMat uMat, int i, int i2, @ByVal UMat uMat2);

    @Namespace("cv::cuda")
    public static native void resetDevice();

    @Namespace("cv::instr")
    public static native void resetTrace();

    @Namespace("cv")
    public static native void rotate(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native void rotate(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native void rotate(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    public static native int roundUp(int i, @Cast({"unsigned int"}) int i2);

    @Namespace("cv")
    @Cast({"size_t"})
    public static native long roundUp(@Cast({"size_t"}) long j, @Cast({"unsigned int"}) int i);

    @Namespace("cv")
    public static native void scaleAdd(@ByVal GpuMat gpuMat, double d, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void scaleAdd(@ByVal Mat mat, double d, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void scaleAdd(@ByVal UMat uMat, double d, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    @Cast({"schar"})
    @Name({"saturate_cast<schar>"})
    public static native byte scharSaturateCast(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @Cast({"schar"})
    @Name({"saturate_cast<schar>"})
    public static native byte scharSaturateCast(double d);

    @Namespace("cv")
    @Cast({"schar"})
    @Name({"saturate_cast<schar>"})
    public static native byte scharSaturateCast(float f);

    @Namespace("cv")
    @Cast({"schar"})
    @Name({"saturate_cast<schar>"})
    public static native byte scharSaturateCast(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @Cast({"schar"})
    @Name({"saturate_cast<schar>"})
    public static native byte scharSaturateCast(@Cast({"int64"}) long j);

    @Namespace("cv")
    @Cast({"schar"})
    @Name({"saturate_cast<schar>"})
    public static native byte scharSaturateCast(@ByVal float16_t float16_t2);

    @Namespace("cv")
    @Cast({"schar"})
    @Name({"saturate_cast<schar>"})
    public static native byte scharSaturateCast(@Cast({"ushort"}) short s);

    @Namespace("cv")
    public static native void seqInsertSlice(CvSeq cvSeq, int i, @Const opencv_core.CvArr cvArr);

    @Namespace("cv")
    public static native void seqPop(CvSeq cvSeq);

    @Namespace("cv")
    public static native void seqPop(CvSeq cvSeq, Pointer pointer);

    @Namespace("cv")
    public static native void seqPopFront(CvSeq cvSeq);

    @Namespace("cv")
    public static native void seqPopFront(CvSeq cvSeq, Pointer pointer);

    @Namespace("cv")
    @Cast({"schar*"})
    public static native BytePointer seqPush(CvSeq cvSeq);

    @Namespace("cv")
    @Cast({"schar*"})
    public static native BytePointer seqPush(CvSeq cvSeq, @Const Pointer pointer);

    @Namespace("cv")
    @Cast({"schar*"})
    public static native BytePointer seqPushFront(CvSeq cvSeq);

    @Namespace("cv")
    @Cast({"schar*"})
    public static native BytePointer seqPushFront(CvSeq cvSeq, @Const Pointer pointer);

    @Namespace("cv")
    public static native void seqRemove(CvSeq cvSeq, int i);

    @Namespace("cv")
    public static native void seqRemoveSlice(CvSeq cvSeq, @ByVal CvSlice cvSlice);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean setBreakOnError(@Cast({"bool"}) boolean z);

    @Namespace("cv::cuda")
    public static native void setBufferPoolConfig(int i, @Cast({"size_t"}) long j, int i2);

    @Namespace("cv::cuda")
    public static native void setBufferPoolUsage(@Cast({"bool"}) boolean z);

    @Namespace("cv::cuda")
    public static native void setDevice(int i);

    @Namespace("cv::instr")
    public static native void setFlags(@Cast({"cv::instr::FLAGS"}) int i);

    @Namespace("cv")
    public static native void setIdentity(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void setIdentity(@ByVal GpuMat gpuMat, @ByRef(nullValue = "cv::Scalar(1)") @Const Scalar scalar);

    @Namespace("cv")
    public static native void setIdentity(@ByVal Mat mat);

    @Namespace("cv")
    public static native void setIdentity(@ByVal Mat mat, @ByRef(nullValue = "cv::Scalar(1)") @Const Scalar scalar);

    @Namespace("cv")
    public static native void setIdentity(@ByVal UMat uMat);

    @Namespace("cv")
    public static native void setIdentity(@ByVal UMat uMat, @ByRef(nullValue = "cv::Scalar(1)") @Const Scalar scalar);

    @Namespace("cv::ipp")
    public static native void setIppStatus(int i);

    @Namespace("cv::ipp")
    public static native void setIppStatus(int i, String str, String str2, int i2);

    @Namespace("cv::ipp")
    public static native void setIppStatus(int i, @Cast({"const char*"}) BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, int i2);

    @Namespace("cv")
    public static native void setNumThreads(int i);

    @Namespace("cv")
    public static native void setRNGSeed(int i);

    @Namespace("cv::ipp")
    public static native void setUseIPP(@Cast({"bool"}) boolean z);

    @Namespace("cv::ipp")
    public static native void setUseIPP_NE(@Cast({"bool"}) boolean z);

    @Namespace("cv::ipp")
    public static native void setUseIPP_NotExact(@Cast({"bool"}) boolean z);

    @Namespace("cv::instr")
    public static native void setUseInstrumentation(@Cast({"bool"}) boolean z);

    @Namespace("cv::ocl")
    public static native void setUseOpenCL(@Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void setUseOptimized(@Cast({"bool"}) boolean z);

    @Namespace("cv")
    @opencv_core.Str
    @Name({"operator <<"})
    public static native String shiftLeft(@opencv_core.Str String str, @opencv_core.Ptr Formatted formatted);

    @Namespace("cv")
    @opencv_core.Str
    @Name({"operator <<"})
    public static native String shiftLeft(@opencv_core.Str String str, @ByRef @Const Mat mat);

    @Namespace("cv")
    @opencv_core.Str
    @Name({"operator <<"})
    public static native BytePointer shiftLeft(@opencv_core.Str BytePointer bytePointer, @opencv_core.Ptr Formatted formatted);

    @Namespace("cv")
    @opencv_core.Str
    @Name({"operator <<"})
    public static native BytePointer shiftLeft(@opencv_core.Str BytePointer bytePointer, @ByRef @Const Mat mat);

    @ByRef
    @Cast({"std::ostream*"})
    @Name({"operator <<"})
    @Namespace("cv")
    public static native Pointer shiftLeft(@ByRef @Cast({"std::ostream*"}) Pointer pointer, @ByRef @Const TickMeter tickMeter);

    @Namespace("cv")
    @ByRef
    @Name({"operator <<"})
    public static native FileStorage shiftLeft(@ByRef FileStorage fileStorage, @opencv_core.Str String str);

    @Namespace("cv")
    @ByRef
    @Name({"operator <<"})
    public static native FileStorage shiftLeft(@ByRef FileStorage fileStorage, @Cast({"char*"}) ByteBuffer byteBuffer);

    @Namespace("cv")
    @ByRef
    @Name({"operator <<"})
    public static native FileStorage shiftLeft(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer);

    @Namespace("cv")
    @ByRef
    @Name({"operator <<"})
    public static native FileStorage shiftLeft(@ByRef FileStorage fileStorage, @Cast({"char*"}) byte[] bArr);

    @Namespace("cv")
    @Name({"operator >>"})
    public static native void shiftRight(@ByRef @Const FileNode fileNode, @ByRef DMatch dMatch);

    @Namespace("cv")
    @Name({"operator >>"})
    public static native void shiftRight(@ByRef @Const FileNode fileNode, @ByRef DMatchVector dMatchVector);

    @Namespace("cv")
    @Name({"operator >>"})
    public static native void shiftRight(@ByRef @Const FileNode fileNode, @ByRef KeyPoint keyPoint);

    @Namespace("cv")
    @Name({"operator >>"})
    public static native void shiftRight(@ByRef @Const FileNode fileNode, @ByRef KeyPointVector keyPointVector);

    @Namespace("cv")
    @Name({"saturate_cast<short>"})
    public static native short shortSaturateCast(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @Name({"saturate_cast<short>"})
    public static native short shortSaturateCast(double d);

    @Namespace("cv")
    @Name({"saturate_cast<short>"})
    public static native short shortSaturateCast(float f);

    @Namespace("cv")
    @Name({"saturate_cast<short>"})
    public static native short shortSaturateCast(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @Name({"saturate_cast<short>"})
    public static native short shortSaturateCast(@Cast({"int64"}) long j);

    @Namespace("cv")
    @Name({"saturate_cast<short>"})
    public static native short shortSaturateCast(@ByVal float16_t float16_t2);

    @Namespace("cv")
    @Name({"saturate_cast<short>"})
    public static native short shortSaturateCast(@Cast({"ushort"}) short s);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solve(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solve(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solve(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solve(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solve(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solve(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, int i);

    @Namespace("cv")
    public static native int solveCubic(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native int solveCubic(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native int solveCubic(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native int solveLP(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native int solveLP(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native int solveLP(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native double solvePoly(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native double solvePoly(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native double solvePoly(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native double solvePoly(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native double solvePoly(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv")
    public static native double solvePoly(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    public static native void sort(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native void sort(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native void sort(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    public static native void sortIdx(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

    @Namespace("cv")
    public static native void sortIdx(@ByVal Mat mat, @ByVal Mat mat2, int i);

    @Namespace("cv")
    public static native void sortIdx(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

    @Namespace("cv")
    public static native void split(@ByVal GpuMat gpuMat, @ByVal GpuMatVector gpuMatVector);

    @Namespace("cv")
    public static native void split(@ByVal GpuMat gpuMat, @ByVal MatVector matVector);

    @Namespace("cv")
    public static native void split(@ByVal GpuMat gpuMat, @ByVal UMatVector uMatVector);

    @Namespace("cv")
    public static native void split(@ByVal Mat mat, @ByVal GpuMatVector gpuMatVector);

    @Namespace("cv")
    public static native void split(@ByRef @Const Mat mat, Mat mat2);

    @Namespace("cv")
    public static native void split(@ByVal Mat mat, @ByVal MatVector matVector);

    @Namespace("cv")
    public static native void split(@ByVal Mat mat, @ByVal UMatVector uMatVector);

    @Namespace("cv")
    public static native void split(@ByVal UMat uMat, @ByVal GpuMatVector gpuMatVector);

    @Namespace("cv")
    public static native void split(@ByVal UMat uMat, @ByVal MatVector matVector);

    @Namespace("cv")
    public static native void split(@ByVal UMat uMat, @ByVal UMatVector uMatVector);

    @Namespace("cv::hal")
    public static native void split16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @ByPtrPtr @Cast({"ushort**"}) ShortBuffer shortBuffer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"ushort**"}) PointerPointer pointerPointer, int i, int i2);

    @Namespace("cv::hal")
    public static native void split16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @ByPtrPtr @Cast({"ushort**"}) ShortPointer shortPointer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split16u(@Cast({"const ushort*"}) short[] sArr, @ByPtrPtr @Cast({"ushort**"}) short[] sArr2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split32s(@Const IntBuffer intBuffer, @ByPtrPtr IntBuffer intBuffer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split32s(@Const IntPointer intPointer, @ByPtrPtr IntPointer intPointer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split32s(@Const IntPointer intPointer, @Cast({"int**"}) PointerPointer pointerPointer, int i, int i2);

    @Namespace("cv::hal")
    public static native void split32s(@Const int[] iArr, @ByPtrPtr int[] iArr2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split64s(@Cast({"const int64*"}) LongBuffer longBuffer, @ByPtrPtr @Cast({"int64**"}) LongBuffer longBuffer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split64s(@Cast({"const int64*"}) LongPointer longPointer, @ByPtrPtr @Cast({"int64**"}) LongPointer longPointer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split64s(@Cast({"const int64*"}) LongPointer longPointer, @Cast({"int64**"}) PointerPointer pointerPointer, int i, int i2);

    @Namespace("cv::hal")
    public static native void split64s(@Cast({"const int64*"}) long[] jArr, @ByPtrPtr @Cast({"int64**"}) long[] jArr2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @ByPtrPtr @Cast({"uchar**"}) ByteBuffer byteBuffer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split8u(@Cast({"const uchar*"}) BytePointer bytePointer, @ByPtrPtr @Cast({"uchar**"}) BytePointer bytePointer2, int i, int i2);

    @Namespace("cv::hal")
    public static native void split8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"uchar**"}) PointerPointer pointerPointer, int i, int i2);

    @Namespace("cv::hal")
    public static native void split8u(@Cast({"const uchar*"}) byte[] bArr, @ByPtrPtr @Cast({"uchar**"}) byte[] bArr2, int i, int i2);

    @Namespace("cv::hal")
    public static native void sqrt(@Const DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, int i);

    @Namespace("cv::hal")
    public static native void sqrt(@Const FloatBuffer floatBuffer, FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native void sqrt(@Const DoublePointer doublePointer, DoublePointer doublePointer2, int i);

    @Namespace("cv::hal")
    public static native void sqrt(@Const FloatPointer floatPointer, FloatPointer floatPointer2, int i);

    @Namespace("cv")
    public static native void sqrt(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void sqrt(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void sqrt(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv::hal")
    public static native void sqrt(@Const double[] dArr, double[] dArr2, int i);

    @Namespace("cv::hal")
    public static native void sqrt(@Const float[] fArr, float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native void sqrt32f(@Const FloatBuffer floatBuffer, FloatBuffer floatBuffer2, int i);

    @Namespace("cv::hal")
    public static native void sqrt32f(@Const FloatPointer floatPointer, FloatPointer floatPointer2, int i);

    @Namespace("cv::hal")
    public static native void sqrt32f(@Const float[] fArr, float[] fArr2, int i);

    @Namespace("cv::hal")
    public static native void sqrt64f(@Const DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, int i);

    @Namespace("cv::hal")
    public static native void sqrt64f(@Const DoublePointer doublePointer, DoublePointer doublePointer2, int i);

    @Namespace("cv::hal")
    public static native void sqrt64f(@Const double[] dArr, double[] dArr2, int i);

    @Namespace("cv::hal")
    public static native void sub16s(@Const ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Const ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub16s(@Const ShortPointer shortPointer, @Cast({"size_t"}) long j, @Const ShortPointer shortPointer2, @Cast({"size_t"}) long j2, ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub16s(@Const short[] sArr, @Cast({"size_t"}) long j, @Const short[] sArr2, @Cast({"size_t"}) long j2, short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub16u(@Cast({"const ushort*"}) ShortBuffer shortBuffer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortBuffer shortBuffer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortBuffer shortBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub16u(@Cast({"const ushort*"}) ShortPointer shortPointer, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) ShortPointer shortPointer2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) ShortPointer shortPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub16u(@Cast({"const ushort*"}) short[] sArr, @Cast({"size_t"}) long j, @Cast({"const ushort*"}) short[] sArr2, @Cast({"size_t"}) long j2, @Cast({"ushort*"}) short[] sArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub32f(@Const FloatBuffer floatBuffer, @Cast({"size_t"}) long j, @Const FloatBuffer floatBuffer2, @Cast({"size_t"}) long j2, FloatBuffer floatBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub32f(@Const FloatPointer floatPointer, @Cast({"size_t"}) long j, @Const FloatPointer floatPointer2, @Cast({"size_t"}) long j2, FloatPointer floatPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub32f(@Const float[] fArr, @Cast({"size_t"}) long j, @Const float[] fArr2, @Cast({"size_t"}) long j2, float[] fArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub32s(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j, @Const IntBuffer intBuffer2, @Cast({"size_t"}) long j2, IntBuffer intBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub32s(@Const IntPointer intPointer, @Cast({"size_t"}) long j, @Const IntPointer intPointer2, @Cast({"size_t"}) long j2, IntPointer intPointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub32s(@Const int[] iArr, @Cast({"size_t"}) long j, @Const int[] iArr2, @Cast({"size_t"}) long j2, int[] iArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub64f(@Const DoubleBuffer doubleBuffer, @Cast({"size_t"}) long j, @Const DoubleBuffer doubleBuffer2, @Cast({"size_t"}) long j2, DoubleBuffer doubleBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub64f(@Const DoublePointer doublePointer, @Cast({"size_t"}) long j, @Const DoublePointer doublePointer2, @Cast({"size_t"}) long j2, DoublePointer doublePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub64f(@Const double[] dArr, @Cast({"size_t"}) long j, @Const double[] dArr2, @Cast({"size_t"}) long j2, double[] dArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub8s(@Cast({"const schar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub8s(@Cast({"const schar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const schar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub8s(@Cast({"const schar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const schar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"schar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void sub8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv")
    @Cast({"ptrdiff_t"})
    @Name({"operator -"})
    public static native long subtract(@ByRef @Const FileNodeIterator fileNodeIterator, @ByRef @Const FileNodeIterator fileNodeIterator2);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const Mat mat, @ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const Mat mat, @ByRef @Const Scalar scalar);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const MatExpr matExpr, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const MatExpr matExpr, @ByRef @Const Scalar scalar);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const Scalar scalar, @ByRef @Const Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native MatExpr subtract(@ByRef @Const Scalar scalar, @ByRef @Const MatExpr matExpr);

    @Namespace("cv")
    @ByVal
    @Name({"operator -"})
    public static native Range subtract(@ByRef @Const Range range, int i);

    @Namespace("cv")
    public static native void subtract(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void subtract(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat4, int i);

    @Namespace("cv")
    public static native void subtract(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void subtract(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat4, int i);

    @Namespace("cv")
    public static native void subtract(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void subtract(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat4, int i);

    @Namespace("cv")
    @ByRef
    @Name({"operator -="})
    public static native Mat subtractPut(@ByRef Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByRef
    @Name({"operator -="})
    public static native Mat subtractPut(@ByRef Mat mat, @ByRef @Const Scalar scalar);

    @Namespace("cv")
    @ByVal
    @Name({"sum"})
    public static native Scalar sumElems(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    @ByVal
    @Name({"sum"})
    public static native Scalar sumElems(@ByVal Mat mat);

    @Namespace("cv")
    @ByVal
    @Name({"sum"})
    public static native Scalar sumElems(@ByVal UMat uMat);

    @Namespace("cv")
    public static native void swap(@ByRef Mat mat, @ByRef Mat mat2);

    @Namespace("cv")
    public static native void swap(@ByRef UMat uMat, @ByRef UMat uMat2);

    @Namespace("cv")
    @opencv_core.Str
    public static native String tempfile(String str);

    @Namespace("cv")
    @opencv_core.Str
    public static native BytePointer tempfile();

    @Namespace("cv")
    @opencv_core.Str
    public static native BytePointer tempfile(@Cast({"const char*"}) BytePointer bytePointer);

    @Namespace("cv")
    @ByRef
    public static native RNG theRNG();

    @Namespace("cv")
    @StdString
    public static native String toLowerCase(@StdString String str);

    @Namespace("cv")
    @StdString
    public static native BytePointer toLowerCase(@StdString BytePointer bytePointer);

    @Namespace("cv")
    @StdString
    public static native String toUpperCase(@StdString String str);

    @Namespace("cv")
    @StdString
    public static native BytePointer toUpperCase(@StdString BytePointer bytePointer);

    @Namespace("cv")
    @ByVal
    public static native Scalar trace(@ByVal GpuMat gpuMat);

    @Namespace("cv")
    @ByVal
    public static native Scalar trace(@ByVal Mat mat);

    @Namespace("cv")
    @ByVal
    public static native Scalar trace(@ByVal UMat uMat);

    @Namespace("cv")
    public static native void transform(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void transform(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void transform(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void transpose(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

    @Namespace("cv")
    public static native void transpose(@ByVal Mat mat, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void transpose(@ByVal UMat uMat, @ByVal UMat uMat2);

    @Namespace("cv::ocl")
    @Cast({"const char*"})
    public static native BytePointer typeToStr(int i);

    @Namespace("cv")
    @Cast({"uchar"})
    @Name({"saturate_cast<uchar>"})
    public static native byte ucharSaturateCast(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @Cast({"uchar"})
    @Name({"saturate_cast<uchar>"})
    public static native byte ucharSaturateCast(double d);

    @Namespace("cv")
    @Cast({"uchar"})
    @Name({"saturate_cast<uchar>"})
    public static native byte ucharSaturateCast(float f);

    @Namespace("cv")
    @Cast({"uchar"})
    @Name({"saturate_cast<uchar>"})
    public static native byte ucharSaturateCast(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @Cast({"uchar"})
    @Name({"saturate_cast<uchar>"})
    public static native byte ucharSaturateCast(@Cast({"int64"}) long j);

    @Namespace("cv")
    @Cast({"uchar"})
    @Name({"saturate_cast<uchar>"})
    public static native byte ucharSaturateCast(@ByVal float16_t float16_t2);

    @Namespace("cv")
    @Cast({"uchar"})
    @Name({"saturate_cast<uchar>"})
    public static native byte ucharSaturateCast(@Cast({"ushort"}) short s);

    @Namespace("cv")
    @Cast({"uint64"})
    @Name({"saturate_cast<uint64>"})
    public static native int uint64SaturateCast(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @Cast({"uint64"})
    @Name({"saturate_cast<uint64>"})
    public static native int uint64SaturateCast(double d);

    @Namespace("cv")
    @Cast({"uint64"})
    @Name({"saturate_cast<uint64>"})
    public static native int uint64SaturateCast(float f);

    @Namespace("cv")
    @Cast({"uint64"})
    @Name({"saturate_cast<uint64>"})
    public static native int uint64SaturateCast(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @Cast({"uint64"})
    @Name({"saturate_cast<uint64>"})
    public static native int uint64SaturateCast(@Cast({"int64"}) long j);

    @Namespace("cv")
    @Cast({"uint64"})
    @Name({"saturate_cast<uint64>"})
    public static native int uint64SaturateCast(@ByVal float16_t float16_t2);

    @Namespace("cv")
    @Cast({"uint64"})
    @Name({"saturate_cast<uint64>"})
    public static native int uint64SaturateCast(@Cast({"ushort"}) short s);

    @Namespace("cv::cuda")
    public static native void unregisterPageLocked(@ByRef Mat mat);

    @Namespace("cv")
    @Cast({"unsigned"})
    @Name({"saturate_cast<unsigned>"})
    public static native int unsignedSaturateCast(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @Cast({"unsigned"})
    @Name({"saturate_cast<unsigned>"})
    public static native int unsignedSaturateCast(double d);

    @Namespace("cv")
    @Cast({"unsigned"})
    @Name({"saturate_cast<unsigned>"})
    public static native int unsignedSaturateCast(float f);

    @Namespace("cv")
    @Cast({"unsigned"})
    @Name({"saturate_cast<unsigned>"})
    public static native int unsignedSaturateCast(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @Cast({"unsigned"})
    @Name({"saturate_cast<unsigned>"})
    public static native int unsignedSaturateCast(@Cast({"int64"}) long j);

    @Namespace("cv")
    @Cast({"unsigned"})
    @Name({"saturate_cast<unsigned>"})
    public static native int unsignedSaturateCast(@ByVal float16_t float16_t2);

    @Namespace("cv")
    @Cast({"unsigned"})
    @Name({"saturate_cast<unsigned>"})
    public static native int unsignedSaturateCast(@Cast({"ushort"}) short s);

    @Namespace("cv::ipp")
    @Cast({"bool"})
    public static native boolean useIPP();

    @Namespace("cv::ipp")
    @Cast({"bool"})
    public static native boolean useIPP_NE();

    @Namespace("cv::ipp")
    @Cast({"bool"})
    public static native boolean useIPP_NotExact();

    @Namespace("cv::instr")
    @Cast({"bool"})
    public static native boolean useInstrumentation();

    @Namespace("cv::ocl")
    @Cast({"bool"})
    public static native boolean useOpenCL();

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean useOptimized();

    @Namespace("cv")
    @Cast({"ushort"})
    @Name({"saturate_cast<ushort>"})
    public static native short ushortSaturateCast(@Cast({"uchar"}) byte b);

    @Namespace("cv")
    @Cast({"ushort"})
    @Name({"saturate_cast<ushort>"})
    public static native short ushortSaturateCast(double d);

    @Namespace("cv")
    @Cast({"ushort"})
    @Name({"saturate_cast<ushort>"})
    public static native short ushortSaturateCast(float f);

    @Namespace("cv")
    @Cast({"ushort"})
    @Name({"saturate_cast<ushort>"})
    public static native short ushortSaturateCast(@Cast({"unsigned"}) int i);

    @Namespace("cv")
    @Cast({"ushort"})
    @Name({"saturate_cast<ushort>"})
    public static native short ushortSaturateCast(@Cast({"int64"}) long j);

    @Namespace("cv")
    @Cast({"ushort"})
    @Name({"saturate_cast<ushort>"})
    public static native short ushortSaturateCast(@ByVal float16_t float16_t2);

    @Namespace("cv")
    @Cast({"ushort"})
    @Name({"saturate_cast<ushort>"})
    public static native short ushortSaturateCast(@Cast({"ushort"}) short s);

    @Namespace("cv")
    public static native void vconcat(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

    @Namespace("cv")
    public static native void vconcat(@ByVal GpuMatVector gpuMatVector, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void vconcat(@ByVal GpuMatVector gpuMatVector, @ByVal Mat mat);

    @Namespace("cv")
    public static native void vconcat(@ByVal GpuMatVector gpuMatVector, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void vconcat(@Const Mat mat, @Cast({"size_t"}) long j, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void vconcat(@Const Mat mat, @Cast({"size_t"}) long j, @ByVal Mat mat2);

    @Namespace("cv")
    public static native void vconcat(@Const Mat mat, @Cast({"size_t"}) long j, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void vconcat(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

    @Namespace("cv")
    public static native void vconcat(@ByVal MatVector matVector, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void vconcat(@ByVal MatVector matVector, @ByVal Mat mat);

    @Namespace("cv")
    public static native void vconcat(@ByVal MatVector matVector, @ByVal UMat uMat);

    @Namespace("cv")
    public static native void vconcat(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

    @Namespace("cv")
    public static native void vconcat(@ByVal UMatVector uMatVector, @ByVal GpuMat gpuMat);

    @Namespace("cv")
    public static native void vconcat(@ByVal UMatVector uMatVector, @ByVal Mat mat);

    @Namespace("cv")
    public static native void vconcat(@ByVal UMatVector uMatVector, @ByVal UMat uMat);

    @Namespace("cv::ocl")
    @Cast({"const char*"})
    public static native BytePointer vecopTypeToStr(int i);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, double d);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, float f);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, int i);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, double d);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, float f);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, int i);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, @opencv_core.Str String str2);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, @ByRef @Const DMatch dMatch);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, @ByRef @Const DMatchVector dMatchVector);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, @ByRef @Const KeyPoint keyPoint);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, @ByRef @Const KeyPointVector keyPointVector);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, @ByRef @Const Mat mat);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, @ByRef @Const Range range);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str String str, @ByRef @Const SparseMat sparseMat);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, double d);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, float f);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, int i);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, @ByRef @Const DMatch dMatch);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, @ByRef @Const DMatchVector dMatchVector);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, @ByRef @Const KeyPoint keyPoint);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, @ByRef @Const KeyPointVector keyPointVector);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, @ByRef @Const Mat mat);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, @ByRef @Const Range range);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, @ByRef @Const SparseMat sparseMat);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @ByRef @Const DMatch dMatch);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @ByRef @Const DMatchVector dMatchVector);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @ByRef @Const KeyPoint keyPoint);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @ByRef @Const KeyPointVector keyPointVector);

    @Namespace("cv")
    public static native void write(@ByRef FileStorage fileStorage, @ByRef @Const Range range);

    @Namespace("cv")
    public static native void writeScalar(@ByRef FileStorage fileStorage, double d);

    @Namespace("cv")
    public static native void writeScalar(@ByRef FileStorage fileStorage, float f);

    @Namespace("cv")
    public static native void writeScalar(@ByRef FileStorage fileStorage, int i);

    @Namespace("cv")
    public static native void writeScalar(@ByRef FileStorage fileStorage, @opencv_core.Str String str);

    @Namespace("cv")
    public static native void writeScalar(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer);

    @Namespace("cv")
    @Cast({"cv::AccessFlag"})
    @Name({"operator ^"})
    public static native int xor(@Cast({"const cv::AccessFlag"}) int i, @Cast({"const cv::AccessFlag"}) int i2);

    @Namespace("cv")
    @ByVal
    @Name({"operator ^"})
    public static native MatExpr xor(@ByRef @Const Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByVal
    @Name({"operator ^"})
    public static native MatExpr xor(@ByRef @Const Mat mat, @ByRef @Const Scalar scalar);

    @Namespace("cv")
    @ByVal
    @Name({"operator ^"})
    public static native MatExpr xor(@ByRef @Const Scalar scalar, @ByRef @Const Mat mat);

    @Namespace("cv::hal")
    public static native void xor8u(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) ByteBuffer byteBuffer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void xor8u(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) BytePointer bytePointer3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @Namespace("cv::hal")
    public static native void xor8u(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"const uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2, @Cast({"uchar*"}) byte[] bArr3, @Cast({"size_t"}) long j3, int i, int i2, Pointer pointer);

    @ByRef
    @Cast({"cv::AccessFlag*"})
    @Name({"operator ^="})
    @Namespace("cv")
    public static native IntBuffer xorPut(@ByRef @Cast({"cv::AccessFlag*"}) IntBuffer intBuffer, @Cast({"const cv::AccessFlag"}) int i);

    @ByRef
    @Cast({"cv::AccessFlag*"})
    @Name({"operator ^="})
    @Namespace("cv")
    public static native IntPointer xorPut(@ByRef @Cast({"cv::AccessFlag*"}) IntPointer intPointer, @Cast({"const cv::AccessFlag"}) int i);

    @Namespace("cv")
    @ByRef
    @Name({"operator ^="})
    public static native Mat xorPut(@ByRef Mat mat, @ByRef @Const Mat mat2);

    @Namespace("cv")
    @ByRef
    @Name({"operator ^="})
    public static native Mat xorPut(@ByRef Mat mat, @ByRef @Const Scalar scalar);

    @ByRef
    @Cast({"cv::AccessFlag*"})
    @Name({"operator ^="})
    @Namespace("cv")
    public static native int[] xorPut(@ByRef @Cast({"cv::AccessFlag*"}) int[] iArr, @Cast({"const cv::AccessFlag"}) int i);

    static {
        Loader.load();
    }

    @Name({"std::map<int,double>"})
    public static class IntDoubleMap extends Pointer {
        private native void allocate();

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @Index
        public native double get(int i);

        public native IntDoubleMap put(int i, double d);

        @ByRef
        @Name({"operator="})
        public native IntDoubleMap put(@ByRef IntDoubleMap intDoubleMap);

        public native long size();

        static {
            Loader.load();
        }

        public IntDoubleMap(Pointer p) {
            super(p);
        }

        public IntDoubleMap() {
            allocate();
        }

        public boolean empty() {
            return size() == 0;
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @MemberGetter
            @Name({"operator*().first"})
            public native int first();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            @MemberGetter
            @Name({"operator*().second"})
            public native double second();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }
    }

    @Name({"std::vector<std::vector<char> >"})
    public static class ByteVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @Index(function = "at")
        @Cast({"char"})
        public native byte get(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2);

        public native ByteVectorVector put(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2, byte b);

        @ByRef
        @Name({"operator="})
        public native ByteVectorVector put(@ByRef ByteVectorVector byteVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native void resize(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2);

        public native long size();

        @Index(function = "at")
        public native long size(@Cast({"size_t"}) long j);

        static {
            Loader.load();
        }

        public ByteVectorVector(Pointer p) {
            super(p);
        }

        public ByteVectorVector(byte[]... array) {
            this((long) array.length);
            put(array);
        }

        public ByteVectorVector() {
            allocate();
        }

        public ByteVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public boolean empty(@Cast({"size_t"}) long i) {
            return size(i) == 0;
        }

        public void clear(@Cast({"size_t"}) long i) {
            resize(i, 0);
        }

        public byte[][] get() {
            byte[][] array = new byte[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)][];
            for (int i = 0; i < array.length; i++) {
                array[i] = new byte[(size((long) i) < TTL.MAX_VALUE ? (int) size((long) i) : Integer.MAX_VALUE)];
                for (int j = 0; j < array[i].length; j++) {
                    array[i][j] = get((long) i, (long) j);
                }
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.deepToString(get());
        }

        public ByteVectorVector put(byte[]... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                if (size((long) i) != ((long) array[i].length)) {
                    resize((long) i, (long) array[i].length);
                }
                for (int j = 0; j < array[i].length; j++) {
                    put((long) i, (long) j, array[i][j]);
                }
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<int> >"})
    public static class IntVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native int get(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2);

        public native IntVectorVector put(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2, int i);

        @ByRef
        @Name({"operator="})
        public native IntVectorVector put(@ByRef IntVectorVector intVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native void resize(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2);

        public native long size();

        @Index(function = "at")
        public native long size(@Cast({"size_t"}) long j);

        static {
            Loader.load();
        }

        public IntVectorVector(Pointer p) {
            super(p);
        }

        public IntVectorVector(int[]... array) {
            this((long) array.length);
            put(array);
        }

        public IntVectorVector() {
            allocate();
        }

        public IntVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public boolean empty(@Cast({"size_t"}) long i) {
            return size(i) == 0;
        }

        public void clear(@Cast({"size_t"}) long i) {
            resize(i, 0);
        }

        public int[][] get() {
            int[][] array = new int[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)][];
            for (int i = 0; i < array.length; i++) {
                array[i] = new int[(size((long) i) < TTL.MAX_VALUE ? (int) size((long) i) : Integer.MAX_VALUE)];
                for (int j = 0; j < array[i].length; j++) {
                    array[i][j] = get((long) i, (long) j);
                }
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.deepToString(get());
        }

        public IntVectorVector put(int[]... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                if (size((long) i) != ((long) array[i].length)) {
                    resize((long) i, (long) array[i].length);
                }
                for (int j = 0; j < array[i].length; j++) {
                    put((long) i, (long) j, array[i][j]);
                }
            }
            return this;
        }
    }

    @Name({"std::vector<cv::String>"})
    public static class StringVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @opencv_core.Str
        @Index(function = "at")
        public native BytePointer get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @opencv_core.Str BytePointer bytePointer);

        @Index(function = "at")
        @ValueSetter
        public native StringVector put(@Cast({"size_t"}) long j, @opencv_core.Str String str);

        public native StringVector put(@Cast({"size_t"}) long j, BytePointer bytePointer);

        @ByRef
        @Name({"operator="})
        public native StringVector put(@ByRef StringVector stringVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public StringVector(Pointer p) {
            super(p);
        }

        public StringVector(BytePointer value) {
            this(1);
            put(0, value);
        }

        public StringVector(BytePointer... array) {
            this((long) array.length);
            put(array);
        }

        public StringVector(String value) {
            this(1);
            put(0, value);
        }

        public StringVector(String... array) {
            this((long) array.length);
            put(array);
        }

        public StringVector() {
            allocate();
        }

        public StringVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @opencv_core.Str
            @Name({"operator*"})
            public native BytePointer get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public BytePointer[] get() {
            BytePointer[] array = new BytePointer[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public BytePointer pop_back() {
            long size = size();
            BytePointer value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public StringVector push_back(BytePointer value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public StringVector put(BytePointer value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public StringVector put(BytePointer... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }

        public StringVector push_back(String value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public StringVector put(String value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public StringVector put(String... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Point>"})
    public static class PointVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Point get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Point point);

        public native PointVector put(@Cast({"size_t"}) long j, Point point);

        @ByRef
        @Name({"operator="})
        public native PointVector put(@ByRef PointVector pointVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public PointVector(Pointer p) {
            super(p);
        }

        public PointVector(Point value) {
            this(1);
            put(0, value);
        }

        public PointVector(Point... array) {
            this((long) array.length);
            put(array);
        }

        public PointVector() {
            allocate();
        }

        public PointVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Point get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Point[] get() {
            Point[] array = new Point[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Point pop_back() {
            long size = size();
            Point value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public PointVector push_back(Point value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public PointVector put(Point value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public PointVector put(Point... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Point2f>"})
    public static class Point2fVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Point2f get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Point2f point2f);

        public native Point2fVector put(@Cast({"size_t"}) long j, Point2f point2f);

        @ByRef
        @Name({"operator="})
        public native Point2fVector put(@ByRef Point2fVector point2fVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public Point2fVector(Pointer p) {
            super(p);
        }

        public Point2fVector(Point2f value) {
            this(1);
            put(0, value);
        }

        public Point2fVector(Point2f... array) {
            this((long) array.length);
            put(array);
        }

        public Point2fVector() {
            allocate();
        }

        public Point2fVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Point2f get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Point2f[] get() {
            Point2f[] array = new Point2f[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Point2f pop_back() {
            long size = size();
            Point2f value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public Point2fVector push_back(Point2f value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public Point2fVector put(Point2f value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public Point2fVector put(Point2f... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Point2d>"})
    public static class Point2dVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Point2d get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Point2d point2d);

        public native Point2dVector put(@Cast({"size_t"}) long j, Point2d point2d);

        @ByRef
        @Name({"operator="})
        public native Point2dVector put(@ByRef Point2dVector point2dVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public Point2dVector(Pointer p) {
            super(p);
        }

        public Point2dVector(Point2d value) {
            this(1);
            put(0, value);
        }

        public Point2dVector(Point2d... array) {
            this((long) array.length);
            put(array);
        }

        public Point2dVector() {
            allocate();
        }

        public Point2dVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Point2d get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Point2d[] get() {
            Point2d[] array = new Point2d[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Point2d pop_back() {
            long size = size();
            Point2d value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public Point2dVector push_back(Point2d value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public Point2dVector put(Point2d value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public Point2dVector put(Point2d... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Point3i>"})
    public static class Point3iVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Point3i get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Point3i point3i);

        public native Point3iVector put(@Cast({"size_t"}) long j, Point3i point3i);

        @ByRef
        @Name({"operator="})
        public native Point3iVector put(@ByRef Point3iVector point3iVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public Point3iVector(Pointer p) {
            super(p);
        }

        public Point3iVector(Point3i value) {
            this(1);
            put(0, value);
        }

        public Point3iVector(Point3i... array) {
            this((long) array.length);
            put(array);
        }

        public Point3iVector() {
            allocate();
        }

        public Point3iVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Point3i get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Point3i[] get() {
            Point3i[] array = new Point3i[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Point3i pop_back() {
            long size = size();
            Point3i value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public Point3iVector push_back(Point3i value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public Point3iVector put(Point3i value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public Point3iVector put(Point3i... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Point3f>"})
    public static class Point3fVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Point3f get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Point3f point3f);

        public native Point3fVector put(@Cast({"size_t"}) long j, Point3f point3f);

        @ByRef
        @Name({"operator="})
        public native Point3fVector put(@ByRef Point3fVector point3fVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public Point3fVector(Pointer p) {
            super(p);
        }

        public Point3fVector(Point3f value) {
            this(1);
            put(0, value);
        }

        public Point3fVector(Point3f... array) {
            this((long) array.length);
            put(array);
        }

        public Point3fVector() {
            allocate();
        }

        public Point3fVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Point3f get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Point3f[] get() {
            Point3f[] array = new Point3f[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Point3f pop_back() {
            long size = size();
            Point3f value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public Point3fVector push_back(Point3f value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public Point3fVector put(Point3f value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public Point3fVector put(Point3f... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Size>"})
    public static class SizeVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Size get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Size size);

        public native SizeVector put(@Cast({"size_t"}) long j, Size size);

        @ByRef
        @Name({"operator="})
        public native SizeVector put(@ByRef SizeVector sizeVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public SizeVector(Pointer p) {
            super(p);
        }

        public SizeVector(Size value) {
            this(1);
            put(0, value);
        }

        public SizeVector(Size... array) {
            this((long) array.length);
            put(array);
        }

        public SizeVector() {
            allocate();
        }

        public SizeVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Size get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Size[] get() {
            Size[] array = new Size[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Size pop_back() {
            long size = size();
            Size value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public SizeVector push_back(Size value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public SizeVector put(Size value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public SizeVector put(Size... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Rect>"})
    public static class RectVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Rect get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Rect rect);

        public native RectVector put(@Cast({"size_t"}) long j, Rect rect);

        @ByRef
        @Name({"operator="})
        public native RectVector put(@ByRef RectVector rectVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public RectVector(Pointer p) {
            super(p);
        }

        public RectVector(Rect value) {
            this(1);
            put(0, value);
        }

        public RectVector(Rect... array) {
            this((long) array.length);
            put(array);
        }

        public RectVector() {
            allocate();
        }

        public RectVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Rect get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Rect[] get() {
            Rect[] array = new Rect[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Rect pop_back() {
            long size = size();
            Rect value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public RectVector push_back(Rect value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public RectVector put(Rect value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public RectVector put(Rect... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Rect2d>"})
    public static class Rect2dVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Rect2d get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Rect2d rect2d);

        public native Rect2dVector put(@Cast({"size_t"}) long j, Rect2d rect2d);

        @ByRef
        @Name({"operator="})
        public native Rect2dVector put(@ByRef Rect2dVector rect2dVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public Rect2dVector(Pointer p) {
            super(p);
        }

        public Rect2dVector(Rect2d value) {
            this(1);
            put(0, value);
        }

        public Rect2dVector(Rect2d... array) {
            this((long) array.length);
            put(array);
        }

        public Rect2dVector() {
            allocate();
        }

        public Rect2dVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Rect2d get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Rect2d[] get() {
            Rect2d[] array = new Rect2d[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Rect2d pop_back() {
            long size = size();
            Rect2d value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public Rect2dVector push_back(Rect2d value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public Rect2dVector put(Rect2d value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public Rect2dVector put(Rect2d... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Scalar>"})
    public static class ScalarVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Scalar get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Scalar scalar);

        public native ScalarVector put(@Cast({"size_t"}) long j, Scalar scalar);

        @ByRef
        @Name({"operator="})
        public native ScalarVector put(@ByRef ScalarVector scalarVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public ScalarVector(Pointer p) {
            super(p);
        }

        public ScalarVector(Scalar value) {
            this(1);
            put(0, value);
        }

        public ScalarVector(Scalar... array) {
            this((long) array.length);
            put(array);
        }

        public ScalarVector() {
            allocate();
        }

        public ScalarVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Scalar get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Scalar[] get() {
            Scalar[] array = new Scalar[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Scalar pop_back() {
            long size = size();
            Scalar value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public ScalarVector push_back(Scalar value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public ScalarVector put(Scalar value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public ScalarVector put(Scalar... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::KeyPoint>"})
    public static class KeyPointVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native KeyPoint get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef KeyPoint keyPoint);

        public native KeyPointVector put(@Cast({"size_t"}) long j, KeyPoint keyPoint);

        @ByRef
        @Name({"operator="})
        public native KeyPointVector put(@ByRef KeyPointVector keyPointVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public KeyPointVector(Pointer p) {
            super(p);
        }

        public KeyPointVector(KeyPoint value) {
            this(1);
            put(0, value);
        }

        public KeyPointVector(KeyPoint... array) {
            this((long) array.length);
            put(array);
        }

        public KeyPointVector() {
            allocate();
        }

        public KeyPointVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native KeyPoint get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public KeyPoint[] get() {
            KeyPoint[] array = new KeyPoint[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public KeyPoint pop_back() {
            long size = size();
            KeyPoint value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public KeyPointVector push_back(KeyPoint value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public KeyPointVector put(KeyPoint value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public KeyPointVector put(KeyPoint... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::DMatch>"})
    public static class DMatchVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native DMatch get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef DMatch dMatch);

        public native DMatchVector put(@Cast({"size_t"}) long j, DMatch dMatch);

        @ByRef
        @Name({"operator="})
        public native DMatchVector put(@ByRef DMatchVector dMatchVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public DMatchVector(Pointer p) {
            super(p);
        }

        public DMatchVector(DMatch value) {
            this(1);
            put(0, value);
        }

        public DMatchVector(DMatch... array) {
            this((long) array.length);
            put(array);
        }

        public DMatchVector() {
            allocate();
        }

        public DMatchVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native DMatch get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public DMatch[] get() {
            DMatch[] array = new DMatch[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public DMatch pop_back() {
            long size = size();
            DMatch value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public DMatchVector push_back(DMatch value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public DMatchVector put(DMatch value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public DMatchVector put(DMatch... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::Point> >"})
    public static class PointVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native PointVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef PointVector pointVector);

        public native PointVectorVector put(@Cast({"size_t"}) long j, PointVector pointVector);

        @ByRef
        @Name({"operator="})
        public native PointVectorVector put(@ByRef PointVectorVector pointVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public PointVectorVector(Pointer p) {
            super(p);
        }

        public PointVectorVector(PointVector value) {
            this(1);
            put(0, value);
        }

        public PointVectorVector(PointVector... array) {
            this((long) array.length);
            put(array);
        }

        public PointVectorVector() {
            allocate();
        }

        public PointVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native PointVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public PointVector[] get() {
            PointVector[] array = new PointVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public PointVector pop_back() {
            long size = size();
            PointVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public PointVectorVector push_back(PointVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public PointVectorVector put(PointVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public PointVectorVector put(PointVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::Point2f> >"})
    public static class Point2fVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Point2fVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Point2fVector point2fVector);

        public native Point2fVectorVector put(@Cast({"size_t"}) long j, Point2fVector point2fVector);

        @ByRef
        @Name({"operator="})
        public native Point2fVectorVector put(@ByRef Point2fVectorVector point2fVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public Point2fVectorVector(Pointer p) {
            super(p);
        }

        public Point2fVectorVector(Point2fVector value) {
            this(1);
            put(0, value);
        }

        public Point2fVectorVector(Point2fVector... array) {
            this((long) array.length);
            put(array);
        }

        public Point2fVectorVector() {
            allocate();
        }

        public Point2fVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Point2fVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Point2fVector[] get() {
            Point2fVector[] array = new Point2fVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Point2fVector pop_back() {
            long size = size();
            Point2fVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public Point2fVectorVector push_back(Point2fVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public Point2fVectorVector put(Point2fVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public Point2fVectorVector put(Point2fVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::Point2d> >"})
    public static class Point2dVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Point2dVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Point2dVector point2dVector);

        public native Point2dVectorVector put(@Cast({"size_t"}) long j, Point2dVector point2dVector);

        @ByRef
        @Name({"operator="})
        public native Point2dVectorVector put(@ByRef Point2dVectorVector point2dVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public Point2dVectorVector(Pointer p) {
            super(p);
        }

        public Point2dVectorVector(Point2dVector value) {
            this(1);
            put(0, value);
        }

        public Point2dVectorVector(Point2dVector... array) {
            this((long) array.length);
            put(array);
        }

        public Point2dVectorVector() {
            allocate();
        }

        public Point2dVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Point2dVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Point2dVector[] get() {
            Point2dVector[] array = new Point2dVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Point2dVector pop_back() {
            long size = size();
            Point2dVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public Point2dVectorVector push_back(Point2dVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public Point2dVectorVector put(Point2dVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public Point2dVectorVector put(Point2dVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::Point3f> >"})
    public static class Point3fVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        @Cast({"std::vector<cv::Point3f>*"})
        public native Point3fVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef @Cast({"std::vector<cv::Point3f>*"}) Point3fVector point3fVector);

        public native Point3fVectorVector put(@Cast({"size_t"}) long j, Point3fVector point3fVector);

        @ByRef
        @Name({"operator="})
        public native Point3fVectorVector put(@ByRef Point3fVectorVector point3fVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public Point3fVectorVector(Pointer p) {
            super(p);
        }

        public Point3fVectorVector(Point3fVector value) {
            this(1);
            put(0, value);
        }

        public Point3fVectorVector(Point3fVector... array) {
            this((long) array.length);
            put(array);
        }

        public Point3fVectorVector() {
            allocate();
        }

        public Point3fVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Cast({"std::vector<cv::Point3f>*"})
            @Const
            @Name({"operator*"})
            public native Point3fVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Point3fVector[] get() {
            Point3fVector[] array = new Point3fVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Point3fVector pop_back() {
            long size = size();
            Point3fVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public Point3fVectorVector push_back(Point3fVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public Point3fVectorVector put(Point3fVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public Point3fVectorVector put(Point3fVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::Rect> >"})
    public static class RectVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native RectVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef RectVector rectVector);

        public native RectVectorVector put(@Cast({"size_t"}) long j, RectVector rectVector);

        @ByRef
        @Name({"operator="})
        public native RectVectorVector put(@ByRef RectVectorVector rectVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public RectVectorVector(Pointer p) {
            super(p);
        }

        public RectVectorVector(RectVector value) {
            this(1);
            put(0, value);
        }

        public RectVectorVector(RectVector... array) {
            this((long) array.length);
            put(array);
        }

        public RectVectorVector() {
            allocate();
        }

        public RectVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native RectVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public RectVector[] get() {
            RectVector[] array = new RectVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public RectVector pop_back() {
            long size = size();
            RectVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public RectVectorVector push_back(RectVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public RectVectorVector put(RectVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public RectVectorVector put(RectVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::KeyPoint> >"})
    public static class KeyPointVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native KeyPointVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef KeyPointVector keyPointVector);

        public native KeyPointVectorVector put(@Cast({"size_t"}) long j, KeyPointVector keyPointVector);

        @ByRef
        @Name({"operator="})
        public native KeyPointVectorVector put(@ByRef KeyPointVectorVector keyPointVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public KeyPointVectorVector(Pointer p) {
            super(p);
        }

        public KeyPointVectorVector(KeyPointVector value) {
            this(1);
            put(0, value);
        }

        public KeyPointVectorVector(KeyPointVector... array) {
            this((long) array.length);
            put(array);
        }

        public KeyPointVectorVector() {
            allocate();
        }

        public KeyPointVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native KeyPointVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public KeyPointVector[] get() {
            KeyPointVector[] array = new KeyPointVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public KeyPointVector pop_back() {
            long size = size();
            KeyPointVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public KeyPointVectorVector push_back(KeyPointVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public KeyPointVectorVector put(KeyPointVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public KeyPointVectorVector put(KeyPointVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::DMatch> >"})
    public static class DMatchVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native DMatchVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef DMatchVector dMatchVector);

        public native DMatchVectorVector put(@Cast({"size_t"}) long j, DMatchVector dMatchVector);

        @ByRef
        @Name({"operator="})
        public native DMatchVectorVector put(@ByRef DMatchVectorVector dMatchVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public DMatchVectorVector(Pointer p) {
            super(p);
        }

        public DMatchVectorVector(DMatchVector value) {
            this(1);
            put(0, value);
        }

        public DMatchVectorVector(DMatchVector... array) {
            this((long) array.length);
            put(array);
        }

        public DMatchVectorVector() {
            allocate();
        }

        public DMatchVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native DMatchVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public DMatchVector[] get() {
            DMatchVector[] array = new DMatchVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public DMatchVector pop_back() {
            long size = size();
            DMatchVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public DMatchVectorVector push_back(DMatchVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public DMatchVectorVector put(DMatchVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public DMatchVectorVector put(DMatchVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Mat>"})
    public static class MatVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native Mat get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef Mat mat);

        public native MatVector put(@Cast({"size_t"}) long j, Mat mat);

        @ByRef
        @Name({"operator="})
        public native MatVector put(@ByRef MatVector matVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public MatVector(Pointer p) {
            super(p);
        }

        public MatVector(Mat value) {
            this(1);
            put(0, value);
        }

        public MatVector(Mat... array) {
            this((long) array.length);
            put(array);
        }

        public MatVector() {
            allocate();
        }

        public MatVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native Mat get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Mat[] get() {
            Mat[] array = new Mat[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public Mat pop_back() {
            long size = size();
            Mat value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public MatVector push_back(Mat value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public MatVector put(Mat value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public MatVector put(Mat... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::UMat>"})
    public static class UMatVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native UMat get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef UMat uMat);

        public native UMatVector put(@Cast({"size_t"}) long j, UMat uMat);

        @ByRef
        @Name({"operator="})
        public native UMatVector put(@ByRef UMatVector uMatVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public UMatVector(Pointer p) {
            super(p);
        }

        public UMatVector(UMat value) {
            this(1);
            put(0, value);
        }

        public UMatVector(UMat... array) {
            this((long) array.length);
            put(array);
        }

        public UMatVector() {
            allocate();
        }

        public UMatVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native UMat get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public UMat[] get() {
            UMat[] array = new UMat[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public UMat pop_back() {
            long size = size();
            UMat value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public UMatVector push_back(UMat value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public UMatVector put(UMat value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public UMatVector put(UMat... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::cuda::GpuMat>"})
    public static class GpuMatVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native GpuMat get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef GpuMat gpuMat);

        public native GpuMatVector put(@Cast({"size_t"}) long j, GpuMat gpuMat);

        @ByRef
        @Name({"operator="})
        public native GpuMatVector put(@ByRef GpuMatVector gpuMatVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public GpuMatVector(Pointer p) {
            super(p);
        }

        public GpuMatVector(GpuMat value) {
            this(1);
            put(0, value);
        }

        public GpuMatVector(GpuMat... array) {
            this((long) array.length);
            put(array);
        }

        public GpuMatVector() {
            allocate();
        }

        public GpuMatVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native GpuMat get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public GpuMat[] get() {
            GpuMat[] array = new GpuMat[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public GpuMat pop_back() {
            long size = size();
            GpuMat value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public GpuMatVector push_back(GpuMat value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public GpuMatVector put(GpuMat value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public GpuMatVector put(GpuMat... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::Mat> >"})
    public static class MatVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native MatVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef MatVector matVector);

        public native MatVectorVector put(@Cast({"size_t"}) long j, MatVector matVector);

        @ByRef
        @Name({"operator="})
        public native MatVectorVector put(@ByRef MatVectorVector matVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public MatVectorVector(Pointer p) {
            super(p);
        }

        public MatVectorVector(MatVector value) {
            this(1);
            put(0, value);
        }

        public MatVectorVector(MatVector... array) {
            this((long) array.length);
            put(array);
        }

        public MatVectorVector() {
            allocate();
        }

        public MatVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native MatVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public MatVector[] get() {
            MatVector[] array = new MatVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public MatVector pop_back() {
            long size = size();
            MatVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public MatVectorVector push_back(MatVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public MatVectorVector put(MatVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public MatVectorVector put(MatVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::pair<int,double> >"})
    public static class IntDoublePairVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native int first(@Cast({"size_t"}) long j);

        public native IntDoublePairVector first(@Cast({"size_t"}) long j, int i);

        @ByRef
        @Name({"operator="})
        public native IntDoublePairVector put(@ByRef IntDoublePairVector intDoublePairVector);

        public native void resize(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native double second(@Cast({"size_t"}) long j);

        public native IntDoublePairVector second(@Cast({"size_t"}) long j, double d);

        public native long size();

        static {
            Loader.load();
        }

        public IntDoublePairVector(Pointer p) {
            super(p);
        }

        public IntDoublePairVector(int[] firstValue, double[] secondValue) {
            this((long) Math.min(firstValue.length, secondValue.length));
            put(firstValue, secondValue);
        }

        public IntDoublePairVector() {
            allocate();
        }

        public IntDoublePairVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public IntDoublePairVector put(int[] firstValue, double[] secondValue) {
            int i = 0;
            while (i < firstValue.length && i < secondValue.length) {
                first((long) i, firstValue[i]);
                second((long) i, secondValue[i]);
                i++;
            }
            return this;
        }
    }

    @Name({"std::vector<std::pair<int,int> >"})
    public static class IntIntPairVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native int first(@Cast({"size_t"}) long j);

        public native IntIntPairVector first(@Cast({"size_t"}) long j, int i);

        @ByRef
        @Name({"operator="})
        public native IntIntPairVector put(@ByRef IntIntPairVector intIntPairVector);

        public native void resize(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native int second(@Cast({"size_t"}) long j);

        public native IntIntPairVector second(@Cast({"size_t"}) long j, int i);

        public native long size();

        static {
            Loader.load();
        }

        public IntIntPairVector(Pointer p) {
            super(p);
        }

        public IntIntPairVector(int[] firstValue, int[] secondValue) {
            this((long) Math.min(firstValue.length, secondValue.length));
            put(firstValue, secondValue);
        }

        public IntIntPairVector() {
            allocate();
        }

        public IntIntPairVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public IntIntPairVector put(int[] firstValue, int[] secondValue) {
            int i = 0;
            while (i < firstValue.length && i < secondValue.length) {
                first((long) i, firstValue[i]);
                second((long) i, secondValue[i]);
                i++;
            }
            return this;
        }
    }

    @Name({"std::vector<std::pair<cv::Mat,uchar> >"})
    public static class MatBytePairVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByRef
        @Index(function = "at")
        public native Mat first(@Cast({"size_t"}) long j);

        public native MatBytePairVector first(@Cast({"size_t"}) long j, Mat mat);

        @ByRef
        @Name({"operator="})
        public native MatBytePairVector put(@ByRef MatBytePairVector matBytePairVector);

        public native void resize(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native byte second(@Cast({"size_t"}) long j);

        public native MatBytePairVector second(@Cast({"size_t"}) long j, byte b);

        public native long size();

        static {
            Loader.load();
        }

        public MatBytePairVector(Pointer p) {
            super(p);
        }

        public MatBytePairVector(Mat[] firstValue, byte[] secondValue) {
            this((long) Math.min(firstValue.length, secondValue.length));
            put(firstValue, secondValue);
        }

        public MatBytePairVector() {
            allocate();
        }

        public MatBytePairVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public MatBytePairVector put(Mat[] firstValue, byte[] secondValue) {
            int i = 0;
            while (i < firstValue.length && i < secondValue.length) {
                first((long) i, firstValue[i]);
                second((long) i, secondValue[i]);
                i++;
            }
            return this;
        }
    }

    @Name({"std::vector<std::pair<cv::UMat,uchar> >"})
    public static class UMatBytePairVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByRef
        @Index(function = "at")
        public native UMat first(@Cast({"size_t"}) long j);

        public native UMatBytePairVector first(@Cast({"size_t"}) long j, UMat uMat);

        @ByRef
        @Name({"operator="})
        public native UMatBytePairVector put(@ByRef UMatBytePairVector uMatBytePairVector);

        public native void resize(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native byte second(@Cast({"size_t"}) long j);

        public native UMatBytePairVector second(@Cast({"size_t"}) long j, byte b);

        public native long size();

        static {
            Loader.load();
        }

        public UMatBytePairVector(Pointer p) {
            super(p);
        }

        public UMatBytePairVector(UMat[] firstValue, byte[] secondValue) {
            this((long) Math.min(firstValue.length, secondValue.length));
            put(firstValue, secondValue);
        }

        public UMatBytePairVector() {
            allocate();
        }

        public UMatBytePairVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public UMatBytePairVector put(UMat[] firstValue, byte[] secondValue) {
            int i = 0;
            while (i < firstValue.length && i < secondValue.length) {
                first((long) i, firstValue[i]);
                second((long) i, secondValue[i]);
                i++;
            }
            return this;
        }
    }

    @Name({"std::vector<cv::instr::NodeDataTls*>"})
    public static class NodeDataTlsVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @Index(function = "at")
        public native NodeDataTls get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, NodeDataTls nodeDataTls);

        public native NodeDataTlsVector put(@Cast({"size_t"}) long j, NodeDataTls nodeDataTls);

        @ByRef
        @Name({"operator="})
        public native NodeDataTlsVector put(@ByRef NodeDataTlsVector nodeDataTlsVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public NodeDataTlsVector(Pointer p) {
            super(p);
        }

        public NodeDataTlsVector(NodeDataTls value) {
            this(1);
            put(0, value);
        }

        public NodeDataTlsVector(NodeDataTls... array) {
            this((long) array.length);
            put(array);
        }

        public NodeDataTlsVector() {
            allocate();
        }

        public NodeDataTlsVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @Const
            @Name({"operator*"})
            public native NodeDataTls get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public NodeDataTls[] get() {
            NodeDataTls[] array = new NodeDataTls[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return java.util.Arrays.toString(get());
        }

        public NodeDataTls pop_back() {
            long size = size();
            NodeDataTls value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public NodeDataTlsVector push_back(NodeDataTls value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public NodeDataTlsVector put(NodeDataTls value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public NodeDataTlsVector put(NodeDataTls... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::pair<int,int>"})
    @NoOffset
    public static class IntIntPair extends Pointer {
        private native void allocate();

        @MemberGetter
        public native int first();

        public native IntIntPair first(int i);

        @ByRef
        @Name({"operator="})
        public native IntIntPair put(@ByRef IntIntPair intIntPair);

        @MemberGetter
        public native int second();

        public native IntIntPair second(int i);

        static {
            Loader.load();
        }

        public IntIntPair(Pointer p) {
            super(p);
        }

        public IntIntPair(int firstValue, int secondValue) {
            this();
            put(firstValue, secondValue);
        }

        public IntIntPair() {
            allocate();
        }

        public IntIntPair put(int firstValue, int secondValue) {
            first(firstValue);
            second(secondValue);
            return this;
        }
    }

    public static class Cv16suf extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native Cv16suf i(short s);

        public native short i();

        public native Cv16suf u(short s);

        @Cast({"ushort"})
        public native short u();

        static {
            Loader.load();
        }

        public Cv16suf() {
            super((Pointer) null);
            allocate();
        }

        public Cv16suf(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Cv16suf(Pointer p) {
            super(p);
        }

        public Cv16suf position(long position) {
            return (Cv16suf) super.position(position);
        }
    }

    public static class Cv32suf extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native float f();

        public native Cv32suf f(float f);

        public native int i();

        public native Cv32suf i(int i);

        @Cast({"unsigned"})
        public native int u();

        public native Cv32suf u(int i);

        static {
            Loader.load();
        }

        public Cv32suf() {
            super((Pointer) null);
            allocate();
        }

        public Cv32suf(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Cv32suf(Pointer p) {
            super(p);
        }

        public Cv32suf position(long position) {
            return (Cv32suf) super.position(position);
        }
    }

    public static class Cv64suf extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native double f();

        public native Cv64suf f(double d);

        @Cast({"int64"})
        public native long i();

        public native Cv64suf i(long j);

        @Cast({"uint64"})
        public native int u();

        public native Cv64suf u(int i);

        static {
            Loader.load();
        }

        public Cv64suf() {
            super((Pointer) null);
            allocate();
        }

        public Cv64suf(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Cv64suf(Pointer p) {
            super(p);
        }

        public Cv64suf position(long position) {
            return (Cv64suf) super.position(position);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class float16_t extends Pointer {
        @ByVal
        @Name({"zero"})
        public static native float16_t _zero();

        private native void allocate();

        private native void allocate(float f);

        private native void allocateArray(long j);

        @ByVal
        public static native float16_t fromBits(@Cast({"ushort"}) short s);

        @Name({"operator float"})
        public native float asFloat();

        @Cast({"ushort"})
        public native short bits();

        static {
            Loader.load();
        }

        public float16_t(Pointer p) {
            super(p);
        }

        public float16_t(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public float16_t position(long position) {
            return (float16_t) super.position(position);
        }

        public float16_t() {
            super((Pointer) null);
            allocate();
        }

        public float16_t(float x) {
            super((Pointer) null);
            allocate(x);
        }
    }

    @Namespace("cv::hal")
    public static class DFT1D extends Pointer {
        @opencv_core.Ptr
        public static native DFT1D create(int i, int i2, int i3, int i4);

        @opencv_core.Ptr
        public static native DFT1D create(int i, int i2, int i3, int i4, @Cast({"bool*"}) BoolPointer boolPointer);

        @opencv_core.Ptr
        public static native DFT1D create(int i, int i2, int i3, int i4, @Cast({"bool*"}) boolean[] zArr);

        public native void apply(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"uchar*"}) ByteBuffer byteBuffer2);

        public native void apply(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"uchar*"}) BytePointer bytePointer2);

        public native void apply(@Cast({"const uchar*"}) byte[] bArr, @Cast({"uchar*"}) byte[] bArr2);

        static {
            Loader.load();
        }

        public DFT1D(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::hal")
    public static class DFT2D extends Pointer {
        @opencv_core.Ptr
        public static native DFT2D create(int i, int i2, int i3, int i4, int i5, int i6);

        @opencv_core.Ptr
        public static native DFT2D create(int i, int i2, int i3, int i4, int i5, int i6, int i7);

        public native void apply(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2);

        public native void apply(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2);

        public native void apply(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2);

        static {
            Loader.load();
        }

        public DFT2D(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::hal")
    public static class DCT2D extends Pointer {
        @opencv_core.Ptr
        public static native DCT2D create(int i, int i2, int i3, int i4);

        public native void apply(@Cast({"const uchar*"}) ByteBuffer byteBuffer, @Cast({"size_t"}) long j, @Cast({"uchar*"}) ByteBuffer byteBuffer2, @Cast({"size_t"}) long j2);

        public native void apply(@Cast({"const uchar*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"uchar*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2);

        public native void apply(@Cast({"const uchar*"}) byte[] bArr, @Cast({"size_t"}) long j, @Cast({"uchar*"}) byte[] bArr2, @Cast({"size_t"}) long j2);

        static {
            Loader.load();
        }

        public DCT2D(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class Hamming extends Pointer {
        public static final int normType = normType();

        private native void allocate();

        private native void allocateArray(long j);

        @MemberGetter
        @Cast({"const cv::NormTypes"})
        public static native int normType();

        @Cast({"cv::Hamming::ResultType"})
        @Name({"operator ()"})
        public native int apply(@Cast({"const unsigned char*"}) ByteBuffer byteBuffer, @Cast({"const unsigned char*"}) ByteBuffer byteBuffer2, int i);

        @Cast({"cv::Hamming::ResultType"})
        @Name({"operator ()"})
        public native int apply(@Cast({"const unsigned char*"}) BytePointer bytePointer, @Cast({"const unsigned char*"}) BytePointer bytePointer2, int i);

        @Cast({"cv::Hamming::ResultType"})
        @Name({"operator ()"})
        public native int apply(@Cast({"const unsigned char*"}) byte[] bArr, @Cast({"const unsigned char*"}) byte[] bArr2, int i);

        static {
            Loader.load();
        }

        public Hamming() {
            super((Pointer) null);
            allocate();
        }

        public Hamming(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Hamming(Pointer p) {
            super(p);
        }

        public Hamming position(long position) {
            return (Hamming) super.position(position);
        }
    }

    @Namespace("cv::ogl")
    @Opaque
    public static class Buffer extends Pointer {
        public Buffer() {
            super((Pointer) null);
        }

        public Buffer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ogl")
    @Opaque
    public static class Texture2D extends Pointer {
        public Texture2D() {
            super((Pointer) null);
        }

        public Texture2D(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ogl")
    @Opaque
    public static class Arrays extends Pointer {
        public Arrays() {
            super((Pointer) null);
        }

        public Arrays(Pointer p) {
            super(p);
        }
    }

    public static class ErrorCallback extends FunctionPointer {
        private native void allocate();

        public native int call(int i, @Cast({"const char*"}) BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"const char*"}) BytePointer bytePointer3, int i2, Pointer pointer);

        static {
            Loader.load();
        }

        public ErrorCallback(Pointer p) {
            super(p);
        }

        protected ErrorCallback() {
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TickMeter extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"int64"})
        public native long getCounter();

        public native double getTimeMicro();

        public native double getTimeMilli();

        public native double getTimeSec();

        @Cast({"int64"})
        public native long getTimeTicks();

        public native void reset();

        public native void start();

        public native void stop();

        static {
            Loader.load();
        }

        public TickMeter(Pointer p) {
            super(p);
        }

        public TickMeter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TickMeter position(long position) {
            return (TickMeter) super.position(position);
        }

        public TickMeter() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    public static class ParallelLoopBody extends Pointer {
        @Name({"operator ()"})
        public native void apply(@ByRef @Const Range range);

        static {
            Loader.load();
        }

        public ParallelLoopBody(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class ParallelLoopBodyLambdaWrapper extends ParallelLoopBody {
        private native void allocate(@ByVal opencv_core.Functor functor);

        @Name({"operator ()"})
        public native void apply(@ByRef @Const Range range);

        static {
            Loader.load();
        }

        public ParallelLoopBodyLambdaWrapper(Pointer p) {
            super(p);
        }

        public ParallelLoopBodyLambdaWrapper(@ByVal opencv_core.Functor functor) {
            super((Pointer) null);
            allocate(functor);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TLSDataContainer extends Pointer {
        public native void cleanup();

        static {
            Loader.load();
        }

        public TLSDataContainer(Pointer p) {
            super(p);
        }
    }

    @Name({"cv::TLSData<cv::instr::NodeDataTls>"})
    public static class NodeDataTlsData extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native void cleanup();

        public native void gather(@ByRef NodeDataTlsVector nodeDataTlsVector);

        public native NodeDataTls get();

        @ByRef
        public native NodeDataTls getRef();

        static {
            Loader.load();
        }

        public NodeDataTlsData(Pointer p) {
            super(p);
        }

        public NodeDataTlsData(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NodeDataTlsData position(long position) {
            return (NodeDataTlsData) super.position(position);
        }

        public NodeDataTlsData() {
            super((Pointer) null);
            allocate();
        }
    }

    @Name({"cv::Node<cv::instr::NodeData>"})
    @NoOffset
    public static class InstrNode extends Pointer {
        private native void allocate();

        private native void allocate(@ByRef NodeData nodeData);

        private native void allocateArray(long j);

        public native void addChild(InstrNode instrNode);

        public native int findChild(InstrNode instrNode);

        public native InstrNode findChild(@ByRef NodeData nodeData);

        public native int getDepth();

        @Cast({"cv::Node<cv::instr::NodeData>**"})
        @StdVector
        public native PointerPointer m_childs();

        public native InstrNode m_childs(PointerPointer pointerPointer);

        public native InstrNode m_pParent();

        public native InstrNode m_pParent(InstrNode instrNode);

        public native InstrNode m_payload(NodeData nodeData);

        @ByRef
        public native NodeData m_payload();

        public native void removeChilds();

        static {
            Loader.load();
        }

        public InstrNode(Pointer p) {
            super(p);
        }

        public InstrNode(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public InstrNode position(long position) {
            return (InstrNode) super.position(position);
        }

        public InstrNode() {
            super((Pointer) null);
            allocate();
        }

        public InstrNode(@ByRef NodeData payload) {
            super((Pointer) null);
            allocate(payload);
        }
    }

    @Namespace("cv::instr")
    @NoOffset
    public static class NodeDataTls extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"uint64"})
        public native int m_ticksTotal();

        public native NodeDataTls m_ticksTotal(int i);

        static {
            Loader.load();
        }

        public NodeDataTls(Pointer p) {
            super(p);
        }

        public NodeDataTls(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NodeDataTls position(long position) {
            return (NodeDataTls) super.position(position);
        }

        public NodeDataTls() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::instr")
    @NoOffset
    public static class NodeData extends Pointer {
        private native void allocate();

        private native void allocate(String str, String str2, int i, Pointer pointer, @Cast({"bool"}) boolean z, @Cast({"cv::instr::TYPE"}) int i2, @Cast({"cv::instr::IMPL"}) int i3);

        private native void allocate(@Cast({"const char*"}) BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, int i, Pointer pointer, @Cast({"bool"}) boolean z, @Cast({"cv::instr::TYPE"}) int i2, @Cast({"cv::instr::IMPL"}) int i3);

        private native void allocate(@ByRef NodeData nodeData);

        private native void allocateArray(long j);

        public native double getMeanMs();

        public native double getTotalMs();

        public native NodeData m_alwaysExpand(boolean z);

        @Cast({"bool"})
        public native boolean m_alwaysExpand();

        public native int m_counter();

        public native NodeData m_counter(int i);

        @MemberGetter
        @Cast({"const char*"})
        public native BytePointer m_fileName();

        public native NodeData m_funError(boolean z);

        @Cast({"bool"})
        public native boolean m_funError();

        @opencv_core.Str
        public native BytePointer m_funName();

        public native NodeData m_funName(BytePointer bytePointer);

        @Cast({"cv::instr::IMPL"})
        public native int m_implType();

        public native NodeData m_implType(int i);

        @Cast({"cv::instr::TYPE"})
        public native int m_instrType();

        public native NodeData m_instrType(int i);

        public native int m_lineNum();

        public native NodeData m_lineNum(int i);

        public native Pointer m_retAddress();

        public native NodeData m_retAddress(Pointer pointer);

        public native int m_threads();

        public native NodeData m_threads(int i);

        @Cast({"uint64"})
        public native int m_ticksTotal();

        public native NodeData m_ticksTotal(int i);

        @ByRef
        @MemberGetter
        public native NodeDataTlsData m_tls();

        @ByRef
        @Name({"operator ="})
        public native NodeData put(@ByRef @Const NodeData nodeData);

        static {
            Loader.load();
        }

        public NodeData(Pointer p) {
            super(p);
        }

        public NodeData(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NodeData position(long position) {
            return (NodeData) super.position(position);
        }

        public NodeData(@Cast({"const char*"}) BytePointer funName, @Cast({"const char*"}) BytePointer fileName, int lineNum, Pointer retAddress, @Cast({"bool"}) boolean alwaysExpand, @Cast({"cv::instr::TYPE"}) int instrType, @Cast({"cv::instr::IMPL"}) int implType) {
            super((Pointer) null);
            allocate(funName, fileName, lineNum, retAddress, alwaysExpand, instrType, implType);
        }

        public NodeData() {
            super((Pointer) null);
            allocate();
        }

        public NodeData(String funName, String fileName, int lineNum, Pointer retAddress, @Cast({"bool"}) boolean alwaysExpand, @Cast({"cv::instr::TYPE"}) int instrType, @Cast({"cv::instr::IMPL"}) int implType) {
            super((Pointer) null);
            allocate(funName, fileName, lineNum, retAddress, alwaysExpand, instrType, implType);
        }

        public NodeData(@ByRef NodeData ref) {
            super((Pointer) null);
            allocate(ref);
        }
    }

    @NoOffset
    public static class IplImage extends opencv_core.AbstractIplImage {
        private native void allocate();

        private native void allocate(@ByRef @Const Mat mat);

        private native void allocateArray(long j);

        public native int BorderConst(int i);

        @MemberGetter
        public native IntPointer BorderConst();

        public native IplImage BorderConst(int i, int i2);

        public native int BorderMode(int i);

        @MemberGetter
        public native IntPointer BorderMode();

        public native IplImage BorderMode(int i, int i2);

        public native int ID();

        public native IplImage ID(int i);

        public native int align();

        public native IplImage align(int i);

        public native int alphaChannel();

        public native IplImage alphaChannel(int i);

        @Cast({"char"})
        public native byte channelSeq(int i);

        @MemberGetter
        @Cast({"char*"})
        public native BytePointer channelSeq();

        public native IplImage channelSeq(int i, byte b);

        @Cast({"char"})
        public native byte colorModel(int i);

        @MemberGetter
        @Cast({"char*"})
        public native BytePointer colorModel();

        public native IplImage colorModel(int i, byte b);

        public native int dataOrder();

        public native IplImage dataOrder(int i);

        public native int depth();

        public native IplImage depth(int i);

        public native int height();

        public native IplImage height(int i);

        @Cast({"char*"})
        public native BytePointer imageData();

        public native IplImage imageData(BytePointer bytePointer);

        @Cast({"char*"})
        public native BytePointer imageDataOrigin();

        public native IplImage imageDataOrigin(BytePointer bytePointer);

        public native Pointer imageId();

        public native IplImage imageId(Pointer pointer);

        public native int imageSize();

        public native IplImage imageSize(int i);

        public native IplImage maskROI();

        public native IplImage maskROI(IplImage iplImage);

        public native int nChannels();

        public native IplImage nChannels(int i);

        public native int nSize();

        public native IplImage nSize(int i);

        public native int origin();

        public native IplImage origin(int i);

        public native IplImage roi(IplROI iplROI);

        public native IplROI roi();

        public native IplImage tileInfo(IplTileInfo iplTileInfo);

        public native IplTileInfo tileInfo();

        public native int width();

        public native IplImage width(int i);

        public native int widthStep();

        public native IplImage widthStep(int i);

        public /* bridge */ /* synthetic */ Object clone() throws CloneNotSupportedException {
            return super.clone();
        }

        static {
            Loader.load();
        }

        public IplImage(Pointer p) {
            super(p);
        }

        public IplImage(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public IplImage position(long position) {
            return (IplImage) super.position(position);
        }

        public IplImage() {
            super((Pointer) null);
            allocate();
        }

        public IplImage(@ByRef @Const Mat m) {
            super((Pointer) null);
            allocate(m);
        }
    }

    @Opaque
    public static class IplTileInfo extends Pointer {
        public IplTileInfo() {
            super((Pointer) null);
        }

        public IplTileInfo(Pointer p) {
            super(p);
        }
    }

    public static class IplROI extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int coi();

        public native IplROI coi(int i);

        public native int height();

        public native IplROI height(int i);

        public native int width();

        public native IplROI width(int i);

        public native int xOffset();

        public native IplROI xOffset(int i);

        public native int yOffset();

        public native IplROI yOffset(int i);

        static {
            Loader.load();
        }

        public IplROI() {
            super((Pointer) null);
            allocate();
        }

        public IplROI(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public IplROI(Pointer p) {
            super(p);
        }

        public IplROI position(long position) {
            return (IplROI) super.position(position);
        }
    }

    public static class IplConvKernel extends opencv_imgproc.AbstractIplConvKernel {
        private native void allocate();

        private native void allocateArray(long j);

        public native int anchorX();

        public native IplConvKernel anchorX(int i);

        public native int anchorY();

        public native IplConvKernel anchorY(int i);

        public native int nCols();

        public native IplConvKernel nCols(int i);

        public native int nRows();

        public native IplConvKernel nRows(int i);

        public native int nShiftR();

        public native IplConvKernel nShiftR(int i);

        public native IntPointer values();

        public native IplConvKernel values(IntPointer intPointer);

        static {
            Loader.load();
        }

        public IplConvKernel() {
            super((Pointer) null);
            allocate();
        }

        public IplConvKernel(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public IplConvKernel(Pointer p) {
            super(p);
        }

        public IplConvKernel position(long position) {
            return (IplConvKernel) super.position(position);
        }
    }

    public static class IplConvKernelFP extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int anchorX();

        public native IplConvKernelFP anchorX(int i);

        public native int anchorY();

        public native IplConvKernelFP anchorY(int i);

        public native int nCols();

        public native IplConvKernelFP nCols(int i);

        public native int nRows();

        public native IplConvKernelFP nRows(int i);

        public native FloatPointer values();

        public native IplConvKernelFP values(FloatPointer floatPointer);

        static {
            Loader.load();
        }

        public IplConvKernelFP() {
            super((Pointer) null);
            allocate();
        }

        public IplConvKernelFP(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public IplConvKernelFP(Pointer p) {
            super(p);
        }

        public IplConvKernelFP position(long position) {
            return (IplConvKernelFP) super.position(position);
        }
    }

    @NoOffset
    public static class CvMat extends opencv_core.AbstractCvMat {
        private native void allocate();

        private native void allocate(@ByRef @Const Mat mat);

        private native void allocateArray(long j);

        public native int cols();

        public native CvMat cols(int i);

        @Name({"data.db"})
        public native DoublePointer data_db();

        public native CvMat data_db(DoublePointer doublePointer);

        @Name({"data.fl"})
        public native FloatPointer data_fl();

        public native CvMat data_fl(FloatPointer floatPointer);

        @Name({"data.i"})
        public native IntPointer data_i();

        public native CvMat data_i(IntPointer intPointer);

        @Cast({"uchar*"})
        @Name({"data.ptr"})
        public native BytePointer data_ptr();

        public native CvMat data_ptr(BytePointer bytePointer);

        @Name({"data.s"})
        public native ShortPointer data_s();

        public native CvMat data_s(ShortPointer shortPointer);

        public native int hdr_refcount();

        public native CvMat hdr_refcount(int i);

        public native int height();

        public native CvMat height(int i);

        public native IntPointer refcount();

        public native CvMat refcount(IntPointer intPointer);

        public native int rows();

        public native CvMat rows(int i);

        public native int step();

        public native CvMat step(int i);

        public native int type();

        public native CvMat type(int i);

        public native int width();

        public native CvMat width(int i);

        public /* bridge */ /* synthetic */ Object clone() throws CloneNotSupportedException {
            return super.clone();
        }

        static {
            Loader.load();
        }

        public CvMat(Pointer p) {
            super(p);
        }

        public CvMat(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvMat position(long position) {
            return (CvMat) super.position(position);
        }

        public CvMat() {
            super((Pointer) null);
            allocate();
        }

        public CvMat(@ByRef @Const Mat m) {
            super((Pointer) null);
            allocate(m);
        }
    }

    @NoOffset
    public static class CvMatND extends opencv_core.AbstractCvMatND {
        private native void allocate();

        private native void allocate(@ByRef @Const Mat mat);

        private native void allocateArray(long j);

        @Name({"data.db"})
        public native DoublePointer data_db();

        public native CvMatND data_db(DoublePointer doublePointer);

        @Name({"data.fl"})
        public native FloatPointer data_fl();

        public native CvMatND data_fl(FloatPointer floatPointer);

        @Name({"data.i"})
        public native IntPointer data_i();

        public native CvMatND data_i(IntPointer intPointer);

        @Cast({"uchar*"})
        @Name({"data.ptr"})
        public native BytePointer data_ptr();

        public native CvMatND data_ptr(BytePointer bytePointer);

        @Name({"data.s"})
        public native ShortPointer data_s();

        public native CvMatND data_s(ShortPointer shortPointer);

        @Name({"dim", ".size"})
        public native int dim_size(int i);

        public native CvMatND dim_size(int i, int i2);

        @Name({"dim", ".step"})
        public native int dim_step(int i);

        public native CvMatND dim_step(int i, int i2);

        public native int dims();

        public native CvMatND dims(int i);

        public native int hdr_refcount();

        public native CvMatND hdr_refcount(int i);

        public native IntPointer refcount();

        public native CvMatND refcount(IntPointer intPointer);

        public native int type();

        public native CvMatND type(int i);

        public /* bridge */ /* synthetic */ Object clone() throws CloneNotSupportedException {
            return super.clone();
        }

        static {
            Loader.load();
        }

        public CvMatND(Pointer p) {
            super(p);
        }

        public CvMatND(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvMatND position(long position) {
            return (CvMatND) super.position(position);
        }

        public CvMatND() {
            super((Pointer) null);
            allocate();
        }

        public CvMatND(@ByRef @Const Mat m) {
            super((Pointer) null);
            allocate(m);
        }
    }

    public static class CvSparseMat extends opencv_core.AbstractCvSparseMat {
        private native void allocate();

        private native void allocateArray(long j);

        public native void copyToSparseMat(@ByRef SparseMat sparseMat);

        public native int dims();

        public native CvSparseMat dims(int i);

        public native int hashsize();

        public native CvSparseMat hashsize(int i);

        public native Pointer hashtable(int i);

        @Cast({"void**"})
        public native PointerPointer hashtable();

        public native CvSparseMat hashtable(int i, Pointer pointer);

        public native CvSparseMat hashtable(PointerPointer pointerPointer);

        public native int hdr_refcount();

        public native CvSparseMat hdr_refcount(int i);

        public native CvSet heap();

        public native CvSparseMat heap(CvSet cvSet);

        public native int idxoffset();

        public native CvSparseMat idxoffset(int i);

        public native IntPointer refcount();

        public native CvSparseMat refcount(IntPointer intPointer);

        public native int size(int i);

        @MemberGetter
        public native IntPointer size();

        public native CvSparseMat size(int i, int i2);

        public native int type();

        public native CvSparseMat type(int i);

        public native int valoffset();

        public native CvSparseMat valoffset(int i);

        public /* bridge */ /* synthetic */ Object clone() throws CloneNotSupportedException {
            return super.clone();
        }

        static {
            Loader.load();
        }

        public CvSparseMat() {
            super((Pointer) null);
            allocate();
        }

        public CvSparseMat(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSparseMat(Pointer p) {
            super(p);
        }

        public CvSparseMat position(long position) {
            return (CvSparseMat) super.position(position);
        }
    }

    public static class CvSparseNode extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"unsigned"})
        public native int hashval();

        public native CvSparseNode hashval(int i);

        public native CvSparseNode next();

        public native CvSparseNode next(CvSparseNode cvSparseNode);

        static {
            Loader.load();
        }

        public CvSparseNode() {
            super((Pointer) null);
            allocate();
        }

        public CvSparseNode(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSparseNode(Pointer p) {
            super(p);
        }

        public CvSparseNode position(long position) {
            return (CvSparseNode) super.position(position);
        }
    }

    public static class CvSparseMatIterator extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int curidx();

        public native CvSparseMatIterator curidx(int i);

        public native CvSparseMat mat();

        public native CvSparseMatIterator mat(CvSparseMat cvSparseMat);

        public native CvSparseMatIterator node(CvSparseNode cvSparseNode);

        public native CvSparseNode node();

        static {
            Loader.load();
        }

        public CvSparseMatIterator() {
            super((Pointer) null);
            allocate();
        }

        public CvSparseMatIterator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSparseMatIterator(Pointer p) {
            super(p);
        }

        public CvSparseMatIterator position(long position) {
            return (CvSparseMatIterator) super.position(position);
        }
    }

    public static class CvHistogram extends opencv_imgproc.AbstractCvHistogram {
        private native void allocate();

        private native void allocateArray(long j);

        public native opencv_core.CvArr bins();

        public native CvHistogram bins(opencv_core.CvArr cvArr);

        public native CvHistogram mat(CvMatND cvMatND);

        @ByRef
        public native CvMatND mat();

        public native float thresh(int i, int i2);

        @MemberGetter
        @Cast({"float(*)[2]"})
        public native FloatPointer thresh();

        public native CvHistogram thresh(int i, int i2, float f);

        public native FloatPointer thresh2(int i);

        @Cast({"float**"})
        public native PointerPointer thresh2();

        public native CvHistogram thresh2(int i, FloatPointer floatPointer);

        public native CvHistogram thresh2(PointerPointer pointerPointer);

        public native int type();

        public native CvHistogram type(int i);

        static {
            Loader.load();
        }

        public CvHistogram() {
            super((Pointer) null);
            allocate();
        }

        public CvHistogram(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvHistogram(Pointer p) {
            super(p);
        }

        public CvHistogram position(long position) {
            return (CvHistogram) super.position(position);
        }
    }

    @NoOffset
    public static class CvRect extends opencv_core.AbstractCvRect {
        private native void allocate();

        private native void allocate(int i, int i2, int i3, int i4);

        private native void allocateArray(long j);

        public native int height();

        public native CvRect height(int i);

        public native int width();

        public native CvRect width(int i);

        public native int x();

        public native CvRect x(int i);

        public native int y();

        public native CvRect y(int i);

        static {
            Loader.load();
        }

        public CvRect(Pointer p) {
            super(p);
        }

        public CvRect(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvRect position(long position) {
            return (CvRect) super.position(position);
        }

        public CvRect() {
            super((Pointer) null);
            allocate();
        }

        public CvRect(int _x, int _y, int w, int h) {
            super((Pointer) null);
            allocate(_x, _y, w, h);
        }
    }

    @NoOffset
    public static class CvTermCriteria extends Pointer {
        private native void allocate();

        private native void allocate(int i, int i2, double d);

        private native void allocate(@ByRef @Const TermCriteria termCriteria);

        private native void allocateArray(long j);

        @ByVal
        @Name({"operator cv::TermCriteria"})
        public native TermCriteria asTermCriteria();

        public native double epsilon();

        public native CvTermCriteria epsilon(double d);

        public native int max_iter();

        public native CvTermCriteria max_iter(int i);

        public native int type();

        public native CvTermCriteria type(int i);

        static {
            Loader.load();
        }

        public CvTermCriteria(Pointer p) {
            super(p);
        }

        public CvTermCriteria(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvTermCriteria position(long position) {
            return (CvTermCriteria) super.position(position);
        }

        public CvTermCriteria(int _type, int _iter, double _eps) {
            super((Pointer) null);
            allocate(_type, _iter, _eps);
        }

        public CvTermCriteria() {
            super((Pointer) null);
            allocate();
        }

        public CvTermCriteria(@ByRef @Const TermCriteria t) {
            super((Pointer) null);
            allocate(t);
        }
    }

    @NoOffset
    public static class CvPoint extends opencv_core.AbstractCvPoint {
        private native void allocate();

        private native void allocate(int i, int i2);

        private native void allocateArray(long j);

        public native int x();

        public native CvPoint x(int i);

        public native int y();

        public native CvPoint y(int i);

        static {
            Loader.load();
        }

        public CvPoint(Pointer p) {
            super(p);
        }

        public CvPoint(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvPoint position(long position) {
            return (CvPoint) super.position(position);
        }

        public CvPoint() {
            super((Pointer) null);
            allocate();
        }

        public CvPoint(int _x, int _y) {
            super((Pointer) null);
            allocate(_x, _y);
        }
    }

    @NoOffset
    public static class CvPoint2D32f extends opencv_core.AbstractCvPoint2D32f {
        private native void allocate();

        private native void allocate(float f, float f2);

        private native void allocateArray(long j);

        public native float x();

        public native CvPoint2D32f x(float f);

        public native float y();

        public native CvPoint2D32f y(float f);

        static {
            Loader.load();
        }

        public CvPoint2D32f(Pointer p) {
            super(p);
        }

        public CvPoint2D32f(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvPoint2D32f position(long position) {
            return (CvPoint2D32f) super.position(position);
        }

        public CvPoint2D32f() {
            super((Pointer) null);
            allocate();
        }

        public CvPoint2D32f(float _x, float _y) {
            super((Pointer) null);
            allocate(_x, _y);
        }
    }

    @NoOffset
    public static class CvPoint3D32f extends opencv_core.AbstractCvPoint3D32f {
        private native void allocate();

        private native void allocate(float f, float f2, float f3);

        private native void allocateArray(long j);

        public native float x();

        public native CvPoint3D32f x(float f);

        public native float y();

        public native CvPoint3D32f y(float f);

        public native float z();

        public native CvPoint3D32f z(float f);

        static {
            Loader.load();
        }

        public CvPoint3D32f(Pointer p) {
            super(p);
        }

        public CvPoint3D32f(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvPoint3D32f position(long position) {
            return (CvPoint3D32f) super.position(position);
        }

        public CvPoint3D32f() {
            super((Pointer) null);
            allocate();
        }

        public CvPoint3D32f(float _x, float _y, float _z) {
            super((Pointer) null);
            allocate(_x, _y, _z);
        }
    }

    @NoOffset
    public static class CvPoint2D64f extends opencv_core.AbstractCvPoint2D64f {
        private native void allocate();

        private native void allocateArray(long j);

        public native double x();

        public native CvPoint2D64f x(double d);

        public native double y();

        public native CvPoint2D64f y(double d);

        static {
            Loader.load();
        }

        public CvPoint2D64f(Pointer p) {
            super(p);
        }

        public CvPoint2D64f(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvPoint2D64f position(long position) {
            return (CvPoint2D64f) super.position(position);
        }

        public CvPoint2D64f() {
            super((Pointer) null);
            allocate();
        }
    }

    @NoOffset
    public static class CvPoint3D64f extends opencv_core.AbstractCvPoint3D64f {
        private native void allocate();

        private native void allocateArray(long j);

        public native double x();

        public native CvPoint3D64f x(double d);

        public native double y();

        public native CvPoint3D64f y(double d);

        public native double z();

        public native CvPoint3D64f z(double d);

        static {
            Loader.load();
        }

        public CvPoint3D64f(Pointer p) {
            super(p);
        }

        public CvPoint3D64f(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvPoint3D64f position(long position) {
            return (CvPoint3D64f) super.position(position);
        }

        public CvPoint3D64f() {
            super((Pointer) null);
            allocate();
        }
    }

    @NoOffset
    public static class CvSize extends opencv_core.AbstractCvSize {
        private native void allocate();

        private native void allocate(int i, int i2);

        private native void allocateArray(long j);

        public native int height();

        public native CvSize height(int i);

        public native int width();

        public native CvSize width(int i);

        static {
            Loader.load();
        }

        public CvSize(Pointer p) {
            super(p);
        }

        public CvSize(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSize position(long position) {
            return (CvSize) super.position(position);
        }

        public CvSize() {
            super((Pointer) null);
            allocate();
        }

        public CvSize(int w, int h) {
            super((Pointer) null);
            allocate(w, h);
        }
    }

    @NoOffset
    public static class CvSize2D32f extends opencv_core.AbstractCvSize2D32f {
        private native void allocate();

        private native void allocate(float f, float f2);

        private native void allocateArray(long j);

        public native float height();

        public native CvSize2D32f height(float f);

        public native float width();

        public native CvSize2D32f width(float f);

        static {
            Loader.load();
        }

        public CvSize2D32f(Pointer p) {
            super(p);
        }

        public CvSize2D32f(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSize2D32f position(long position) {
            return (CvSize2D32f) super.position(position);
        }

        public CvSize2D32f() {
            super((Pointer) null);
            allocate();
        }

        public CvSize2D32f(float w, float h) {
            super((Pointer) null);
            allocate(w, h);
        }
    }

    @NoOffset
    public static class CvBox2D extends opencv_core.AbstractCvBox2D {
        private native void allocate();

        private native void allocate(@Cast({"CvPoint2D32f*"}) @ByVal(nullValue = "CvPoint2D32f()") FloatBuffer floatBuffer, @ByVal(nullValue = "CvSize2D32f()") CvSize2D32f cvSize2D32f, float f);

        private native void allocate(@ByVal(nullValue = "CvPoint2D32f()") CvPoint2D32f cvPoint2D32f, @ByVal(nullValue = "CvSize2D32f()") CvSize2D32f cvSize2D32f, float f);

        private native void allocate(@ByRef @Const RotatedRect rotatedRect);

        private native void allocate(@Cast({"CvPoint2D32f*"}) @ByVal(nullValue = "CvPoint2D32f()") float[] fArr, @ByVal(nullValue = "CvSize2D32f()") CvSize2D32f cvSize2D32f, float f);

        private native void allocateArray(long j);

        public native float angle();

        public native CvBox2D angle(float f);

        @ByVal
        @Name({"operator cv::RotatedRect"})
        public native RotatedRect asRotatedRect();

        public native CvBox2D center(CvPoint2D32f cvPoint2D32f);

        @ByRef
        public native CvPoint2D32f center();

        public native CvBox2D size(CvSize2D32f cvSize2D32f);

        @ByRef
        public native CvSize2D32f size();

        static {
            Loader.load();
        }

        public CvBox2D(Pointer p) {
            super(p);
        }

        public CvBox2D(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvBox2D position(long position) {
            return (CvBox2D) super.position(position);
        }

        public CvBox2D(@ByVal(nullValue = "CvPoint2D32f()") CvPoint2D32f c, @ByVal(nullValue = "CvSize2D32f()") CvSize2D32f s, float a) {
            super((Pointer) null);
            allocate(c, s, a);
        }

        public CvBox2D() {
            super((Pointer) null);
            allocate();
        }

        public CvBox2D(@Cast({"CvPoint2D32f*"}) @ByVal(nullValue = "CvPoint2D32f()") FloatBuffer c, @ByVal(nullValue = "CvSize2D32f()") CvSize2D32f s, float a) {
            super((Pointer) null);
            allocate(c, s, a);
        }

        public CvBox2D(@Cast({"CvPoint2D32f*"}) @ByVal(nullValue = "CvPoint2D32f()") float[] c, @ByVal(nullValue = "CvSize2D32f()") CvSize2D32f s, float a) {
            super((Pointer) null);
            allocate(c, s, a);
        }

        public CvBox2D(@ByRef @Const RotatedRect rr) {
            super((Pointer) null);
            allocate(rr);
        }
    }

    public static class CvLineIterator extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int err();

        public native CvLineIterator err(int i);

        public native int minus_delta();

        public native CvLineIterator minus_delta(int i);

        public native int minus_step();

        public native CvLineIterator minus_step(int i);

        public native int plus_delta();

        public native CvLineIterator plus_delta(int i);

        public native int plus_step();

        public native CvLineIterator plus_step(int i);

        @Cast({"uchar*"})
        public native BytePointer ptr();

        public native CvLineIterator ptr(BytePointer bytePointer);

        static {
            Loader.load();
        }

        public CvLineIterator() {
            super((Pointer) null);
            allocate();
        }

        public CvLineIterator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvLineIterator(Pointer p) {
            super(p);
        }

        public CvLineIterator position(long position) {
            return (CvLineIterator) super.position(position);
        }
    }

    @NoOffset
    public static class CvSlice extends Pointer {
        private native void allocate();

        private native void allocate(int i, int i2);

        private native void allocate(@ByRef @Const Range range);

        private native void allocateArray(long j);

        @ByVal
        @Name({"operator cv::Range"})
        public native Range asRange();

        public native int end_index();

        public native CvSlice end_index(int i);

        public native int start_index();

        public native CvSlice start_index(int i);

        static {
            Loader.load();
        }

        public CvSlice(Pointer p) {
            super(p);
        }

        public CvSlice(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSlice position(long position) {
            return (CvSlice) super.position(position);
        }

        public CvSlice() {
            super((Pointer) null);
            allocate();
        }

        public CvSlice(int start, int end) {
            super((Pointer) null);
            allocate(start, end);
        }

        public CvSlice(@ByRef @Const Range r) {
            super((Pointer) null);
            allocate(r);
        }
    }

    @NoOffset
    public static class CvScalar extends opencv_core.AbstractCvScalar {
        private native void allocate();

        private native void allocate(double d);

        private native void allocate(double d, double d2, double d3, double d4);

        private native void allocateArray(long j);

        public native double val(int i);

        @MemberGetter
        public native DoublePointer val();

        public native CvScalar val(int i, double d);

        static {
            Loader.load();
        }

        public CvScalar(Pointer p) {
            super(p);
        }

        public CvScalar(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvScalar position(long position) {
            return (CvScalar) super.position(position);
        }

        public CvScalar() {
            super((Pointer) null);
            allocate();
        }

        public CvScalar(double d0, double d1, double d2, double d3) {
            super((Pointer) null);
            allocate(d0, d1, d2, d3);
        }

        public CvScalar(double d0) {
            super((Pointer) null);
            allocate(d0);
        }
    }

    public static class CvMemBlock extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native CvMemBlock next();

        public native CvMemBlock next(CvMemBlock cvMemBlock);

        public native CvMemBlock prev();

        public native CvMemBlock prev(CvMemBlock cvMemBlock);

        static {
            Loader.load();
        }

        public CvMemBlock() {
            super((Pointer) null);
            allocate();
        }

        public CvMemBlock(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvMemBlock(Pointer p) {
            super(p);
        }

        public CvMemBlock position(long position) {
            return (CvMemBlock) super.position(position);
        }
    }

    public static class CvMemStorage extends opencv_core.AbstractCvMemStorage {
        private native void allocate();

        private native void allocateArray(long j);

        public native int block_size();

        public native CvMemStorage block_size(int i);

        public native CvMemBlock bottom();

        public native CvMemStorage bottom(CvMemBlock cvMemBlock);

        public native int free_space();

        public native CvMemStorage free_space(int i);

        public native CvMemStorage parent();

        public native CvMemStorage parent(CvMemStorage cvMemStorage);

        public native int signature();

        public native CvMemStorage signature(int i);

        public native CvMemBlock top();

        public native CvMemStorage top(CvMemBlock cvMemBlock);

        static {
            Loader.load();
        }

        public CvMemStorage() {
            super((Pointer) null);
            allocate();
        }

        public CvMemStorage(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvMemStorage(Pointer p) {
            super(p);
        }

        public CvMemStorage position(long position) {
            return (CvMemStorage) super.position(position);
        }
    }

    public static class CvMemStoragePos extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int free_space();

        public native CvMemStoragePos free_space(int i);

        public native CvMemBlock top();

        public native CvMemStoragePos top(CvMemBlock cvMemBlock);

        static {
            Loader.load();
        }

        public CvMemStoragePos() {
            super((Pointer) null);
            allocate();
        }

        public CvMemStoragePos(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvMemStoragePos(Pointer p) {
            super(p);
        }

        public CvMemStoragePos position(long position) {
            return (CvMemStoragePos) super.position(position);
        }
    }

    public static class CvSeqBlock extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int count();

        public native CvSeqBlock count(int i);

        @Cast({"schar*"})
        public native BytePointer data();

        public native CvSeqBlock data(BytePointer bytePointer);

        public native CvSeqBlock next();

        public native CvSeqBlock next(CvSeqBlock cvSeqBlock);

        public native CvSeqBlock prev();

        public native CvSeqBlock prev(CvSeqBlock cvSeqBlock);

        public native int start_index();

        public native CvSeqBlock start_index(int i);

        static {
            Loader.load();
        }

        public CvSeqBlock() {
            super((Pointer) null);
            allocate();
        }

        public CvSeqBlock(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSeqBlock(Pointer p) {
            super(p);
        }

        public CvSeqBlock position(long position) {
            return (CvSeqBlock) super.position(position);
        }
    }

    public static class CvSeq extends opencv_core.AbstractCvSeq {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"schar*"})
        public native BytePointer block_max();

        public native CvSeq block_max(BytePointer bytePointer);

        public native int delta_elems();

        public native CvSeq delta_elems(int i);

        public native int elem_size();

        public native CvSeq elem_size(int i);

        public native CvSeq first(CvSeqBlock cvSeqBlock);

        public native CvSeqBlock first();

        public native int flags();

        public native CvSeq flags(int i);

        public native CvSeq free_blocks(CvSeqBlock cvSeqBlock);

        public native CvSeqBlock free_blocks();

        public native CvSeq h_next();

        public native CvSeq h_next(CvSeq cvSeq);

        public native CvSeq h_prev();

        public native CvSeq h_prev(CvSeq cvSeq);

        public native int header_size();

        public native CvSeq header_size(int i);

        @Cast({"schar*"})
        public native BytePointer ptr();

        public native CvSeq ptr(BytePointer bytePointer);

        public native CvMemStorage storage();

        public native CvSeq storage(CvMemStorage cvMemStorage);

        public native int total();

        public native CvSeq total(int i);

        public native CvSeq v_next();

        public native CvSeq v_next(CvSeq cvSeq);

        public native CvSeq v_prev();

        public native CvSeq v_prev(CvSeq cvSeq);

        static {
            Loader.load();
        }

        public CvSeq() {
            super((Pointer) null);
            allocate();
        }

        public CvSeq(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSeq(Pointer p) {
            super(p);
        }

        public CvSeq position(long position) {
            return (CvSeq) super.position(position);
        }
    }

    public static class CvSetElem extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int flags();

        public native CvSetElem flags(int i);

        public native CvSetElem next_free();

        public native CvSetElem next_free(CvSetElem cvSetElem);

        static {
            Loader.load();
        }

        public CvSetElem() {
            super((Pointer) null);
            allocate();
        }

        public CvSetElem(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSetElem(Pointer p) {
            super(p);
        }

        public CvSetElem position(long position) {
            return (CvSetElem) super.position(position);
        }
    }

    public static class CvSet extends opencv_core.AbstractCvSet {
        private native void allocate();

        private native void allocateArray(long j);

        public native int active_count();

        public native CvSet active_count(int i);

        @Cast({"schar*"})
        public native BytePointer block_max();

        public native CvSet block_max(BytePointer bytePointer);

        public native int delta_elems();

        public native CvSet delta_elems(int i);

        public native int elem_size();

        public native CvSet elem_size(int i);

        public native CvSeqBlock first();

        public native CvSet first(CvSeqBlock cvSeqBlock);

        public native int flags();

        public native CvSet flags(int i);

        public native CvSeqBlock free_blocks();

        public native CvSet free_blocks(CvSeqBlock cvSeqBlock);

        public native CvSet free_elems(CvSetElem cvSetElem);

        public native CvSetElem free_elems();

        public native CvSeq h_next();

        public native CvSet h_next(CvSeq cvSeq);

        public native CvSeq h_prev();

        public native CvSet h_prev(CvSeq cvSeq);

        public native int header_size();

        public native CvSet header_size(int i);

        @Cast({"schar*"})
        public native BytePointer ptr();

        public native CvSet ptr(BytePointer bytePointer);

        public native CvMemStorage storage();

        public native CvSet storage(CvMemStorage cvMemStorage);

        public native int total();

        public native CvSet total(int i);

        public native CvSeq v_next();

        public native CvSet v_next(CvSeq cvSeq);

        public native CvSeq v_prev();

        public native CvSet v_prev(CvSeq cvSeq);

        static {
            Loader.load();
        }

        public CvSet() {
            super((Pointer) null);
            allocate();
        }

        public CvSet(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSet(Pointer p) {
            super(p);
        }

        public CvSet position(long position) {
            return (CvSet) super.position(position);
        }
    }

    public static class CvGraphEdge extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int flags();

        public native CvGraphEdge flags(int i);

        @MemberGetter
        @Cast({"CvGraphEdge**"})
        public native PointerPointer next();

        public native CvGraphEdge next(int i);

        public native CvGraphEdge next(int i, CvGraphEdge cvGraphEdge);

        @MemberGetter
        @Cast({"CvGraphVtx**"})
        public native PointerPointer vtx();

        public native CvGraphEdge vtx(int i, CvGraphVtx cvGraphVtx);

        public native CvGraphVtx vtx(int i);

        public native float weight();

        public native CvGraphEdge weight(float f);

        static {
            Loader.load();
        }

        public CvGraphEdge() {
            super((Pointer) null);
            allocate();
        }

        public CvGraphEdge(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvGraphEdge(Pointer p) {
            super(p);
        }

        public CvGraphEdge position(long position) {
            return (CvGraphEdge) super.position(position);
        }
    }

    public static class CvGraphVtx extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native CvGraphEdge first();

        public native CvGraphVtx first(CvGraphEdge cvGraphEdge);

        public native int flags();

        public native CvGraphVtx flags(int i);

        static {
            Loader.load();
        }

        public CvGraphVtx() {
            super((Pointer) null);
            allocate();
        }

        public CvGraphVtx(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvGraphVtx(Pointer p) {
            super(p);
        }

        public CvGraphVtx position(long position) {
            return (CvGraphVtx) super.position(position);
        }
    }

    public static class CvGraphVtx2D extends CvGraphVtx {
        private native void allocate();

        private native void allocateArray(long j);

        public native CvGraphEdge first();

        public native CvGraphVtx2D first(CvGraphEdge cvGraphEdge);

        public native int flags();

        public native CvGraphVtx2D flags(int i);

        public native CvGraphVtx2D ptr(CvPoint2D32f cvPoint2D32f);

        public native CvPoint2D32f ptr();

        static {
            Loader.load();
        }

        public CvGraphVtx2D() {
            super((Pointer) null);
            allocate();
        }

        public CvGraphVtx2D(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvGraphVtx2D(Pointer p) {
            super(p);
        }

        public CvGraphVtx2D position(long position) {
            return (CvGraphVtx2D) super.position(position);
        }
    }

    public static class CvGraph extends opencv_core.AbstractCvGraph {
        private native void allocate();

        private native void allocateArray(long j);

        public native int active_count();

        public native CvGraph active_count(int i);

        @Cast({"schar*"})
        public native BytePointer block_max();

        public native CvGraph block_max(BytePointer bytePointer);

        public native int delta_elems();

        public native CvGraph delta_elems(int i);

        public native CvGraph edges(CvSet cvSet);

        public native CvSet edges();

        public native int elem_size();

        public native CvGraph elem_size(int i);

        public native CvGraph first(CvSeqBlock cvSeqBlock);

        public native CvSeqBlock first();

        public native int flags();

        public native CvGraph flags(int i);

        public native CvGraph free_blocks(CvSeqBlock cvSeqBlock);

        public native CvSeqBlock free_blocks();

        public native CvGraph free_elems(CvSetElem cvSetElem);

        public native CvSetElem free_elems();

        public native CvGraph h_next(CvSeq cvSeq);

        public native CvSeq h_next();

        public native CvGraph h_prev(CvSeq cvSeq);

        public native CvSeq h_prev();

        public native int header_size();

        public native CvGraph header_size(int i);

        @Cast({"schar*"})
        public native BytePointer ptr();

        public native CvGraph ptr(BytePointer bytePointer);

        public native CvGraph storage(CvMemStorage cvMemStorage);

        public native CvMemStorage storage();

        public native int total();

        public native CvGraph total(int i);

        public native CvGraph v_next(CvSeq cvSeq);

        public native CvSeq v_next();

        public native CvGraph v_prev(CvSeq cvSeq);

        public native CvSeq v_prev();

        static {
            Loader.load();
        }

        public CvGraph() {
            super((Pointer) null);
            allocate();
        }

        public CvGraph(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvGraph(Pointer p) {
            super(p);
        }

        public CvGraph position(long position) {
            return (CvGraph) super.position(position);
        }
    }

    public static class CvChain extends CvSeq {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"schar*"})
        public native BytePointer block_max();

        public native CvChain block_max(BytePointer bytePointer);

        public native int delta_elems();

        public native CvChain delta_elems(int i);

        public native int elem_size();

        public native CvChain elem_size(int i);

        public native CvChain first(CvSeqBlock cvSeqBlock);

        public native CvSeqBlock first();

        public native int flags();

        public native CvChain flags(int i);

        public native CvChain free_blocks(CvSeqBlock cvSeqBlock);

        public native CvSeqBlock free_blocks();

        public native CvChain h_next(CvSeq cvSeq);

        public native CvSeq h_next();

        public native CvChain h_prev(CvSeq cvSeq);

        public native CvSeq h_prev();

        public native int header_size();

        public native CvChain header_size(int i);

        public native CvChain origin(CvPoint cvPoint);

        @ByRef
        public native CvPoint origin();

        @Cast({"schar*"})
        public native BytePointer ptr();

        public native CvChain ptr(BytePointer bytePointer);

        public native CvChain storage(CvMemStorage cvMemStorage);

        public native CvMemStorage storage();

        public native int total();

        public native CvChain total(int i);

        public native CvChain v_next(CvSeq cvSeq);

        public native CvSeq v_next();

        public native CvChain v_prev(CvSeq cvSeq);

        public native CvSeq v_prev();

        static {
            Loader.load();
        }

        public CvChain() {
            super((Pointer) null);
            allocate();
        }

        public CvChain(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvChain(Pointer p) {
            super(p);
        }

        public CvChain position(long position) {
            return (CvChain) super.position(position);
        }
    }

    public static class CvContour extends CvSeq {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"schar*"})
        public native BytePointer block_max();

        public native CvContour block_max(BytePointer bytePointer);

        public native int color();

        public native CvContour color(int i);

        public native int delta_elems();

        public native CvContour delta_elems(int i);

        public native int elem_size();

        public native CvContour elem_size(int i);

        public native CvContour first(CvSeqBlock cvSeqBlock);

        public native CvSeqBlock first();

        public native int flags();

        public native CvContour flags(int i);

        public native CvContour free_blocks(CvSeqBlock cvSeqBlock);

        public native CvSeqBlock free_blocks();

        public native CvContour h_next(CvSeq cvSeq);

        public native CvSeq h_next();

        public native CvContour h_prev(CvSeq cvSeq);

        public native CvSeq h_prev();

        public native int header_size();

        public native CvContour header_size(int i);

        @Cast({"schar*"})
        public native BytePointer ptr();

        public native CvContour ptr(BytePointer bytePointer);

        public native CvContour rect(CvRect cvRect);

        @ByRef
        public native CvRect rect();

        public native int reserved(int i);

        @MemberGetter
        public native IntPointer reserved();

        public native CvContour reserved(int i, int i2);

        public native CvContour storage(CvMemStorage cvMemStorage);

        public native CvMemStorage storage();

        public native int total();

        public native CvContour total(int i);

        public native CvContour v_next(CvSeq cvSeq);

        public native CvSeq v_next();

        public native CvContour v_prev(CvSeq cvSeq);

        public native CvSeq v_prev();

        static {
            Loader.load();
        }

        public CvContour() {
            super((Pointer) null);
            allocate();
        }

        public CvContour(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvContour(Pointer p) {
            super(p);
        }

        public CvContour position(long position) {
            return (CvContour) super.position(position);
        }
    }

    public static class CvSeqWriter extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native CvSeqBlock block();

        public native CvSeqWriter block(CvSeqBlock cvSeqBlock);

        @Cast({"schar*"})
        public native BytePointer block_max();

        public native CvSeqWriter block_max(BytePointer bytePointer);

        @Cast({"schar*"})
        public native BytePointer block_min();

        public native CvSeqWriter block_min(BytePointer bytePointer);

        public native int header_size();

        public native CvSeqWriter header_size(int i);

        @Cast({"schar*"})
        public native BytePointer ptr();

        public native CvSeqWriter ptr(BytePointer bytePointer);

        public native CvSeq seq();

        public native CvSeqWriter seq(CvSeq cvSeq);

        static {
            Loader.load();
        }

        public CvSeqWriter() {
            super((Pointer) null);
            allocate();
        }

        public CvSeqWriter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSeqWriter(Pointer p) {
            super(p);
        }

        public CvSeqWriter position(long position) {
            return (CvSeqWriter) super.position(position);
        }
    }

    public static class CvSeqReader extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native CvSeqBlock block();

        public native CvSeqReader block(CvSeqBlock cvSeqBlock);

        @Cast({"schar*"})
        public native BytePointer block_max();

        public native CvSeqReader block_max(BytePointer bytePointer);

        @Cast({"schar*"})
        public native BytePointer block_min();

        public native CvSeqReader block_min(BytePointer bytePointer);

        public native int delta_index();

        public native CvSeqReader delta_index(int i);

        public native int header_size();

        public native CvSeqReader header_size(int i);

        @Cast({"schar*"})
        public native BytePointer prev_elem();

        public native CvSeqReader prev_elem(BytePointer bytePointer);

        @Cast({"schar*"})
        public native BytePointer ptr();

        public native CvSeqReader ptr(BytePointer bytePointer);

        public native CvSeq seq();

        public native CvSeqReader seq(CvSeq cvSeq);

        static {
            Loader.load();
        }

        public CvSeqReader() {
            super((Pointer) null);
            allocate();
        }

        public CvSeqReader(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvSeqReader(Pointer p) {
            super(p);
        }

        public CvSeqReader position(long position) {
            return (CvSeqReader) super.position(position);
        }
    }

    public static class CvNArrayIterator extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int count();

        public native CvNArrayIterator count(int i);

        public native int dims();

        public native CvNArrayIterator dims(int i);

        @MemberGetter
        @Cast({"CvMatND**"})
        public native PointerPointer hdr();

        public native CvMatND hdr(int i);

        public native CvNArrayIterator hdr(int i, CvMatND cvMatND);

        @Cast({"uchar*"})
        public native BytePointer ptr(int i);

        @MemberGetter
        @Cast({"uchar**"})
        public native PointerPointer ptr();

        public native CvNArrayIterator ptr(int i, BytePointer bytePointer);

        public native CvNArrayIterator size(CvSize cvSize);

        @ByRef
        public native CvSize size();

        public native int stack(int i);

        @MemberGetter
        public native IntPointer stack();

        public native CvNArrayIterator stack(int i, int i2);

        static {
            Loader.load();
        }

        public CvNArrayIterator() {
            super((Pointer) null);
            allocate();
        }

        public CvNArrayIterator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvNArrayIterator(Pointer p) {
            super(p);
        }

        public CvNArrayIterator position(long position) {
            return (CvNArrayIterator) super.position(position);
        }
    }

    @Convention("CV_CDECL")
    public static class CvCmpFunc extends FunctionPointer {
        private native void allocate();

        public native int call(@Const Pointer pointer, @Const Pointer pointer2, Pointer pointer3);

        static {
            Loader.load();
        }

        public CvCmpFunc(Pointer p) {
            super(p);
        }

        protected CvCmpFunc() {
            allocate();
        }
    }

    public static class CvGraphScanner extends opencv_core.AbstractCvGraphScanner {
        private native void allocate();

        private native void allocateArray(long j);

        public native CvGraphScanner dst(CvGraphVtx cvGraphVtx);

        public native CvGraphVtx dst();

        public native CvGraphEdge edge();

        public native CvGraphScanner edge(CvGraphEdge cvGraphEdge);

        public native CvGraph graph();

        public native CvGraphScanner graph(CvGraph cvGraph);

        public native int index();

        public native CvGraphScanner index(int i);

        public native int mask();

        public native CvGraphScanner mask(int i);

        public native CvGraphScanner stack(CvSeq cvSeq);

        public native CvSeq stack();

        public native CvGraphScanner vtx(CvGraphVtx cvGraphVtx);

        public native CvGraphVtx vtx();

        static {
            Loader.load();
        }

        public CvGraphScanner() {
            super((Pointer) null);
            allocate();
        }

        public CvGraphScanner(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvGraphScanner(Pointer p) {
            super(p);
        }

        public CvGraphScanner position(long position) {
            return (CvGraphScanner) super.position(position);
        }
    }

    public static class CvTreeNodeIterator extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int level();

        public native CvTreeNodeIterator level(int i);

        public native int max_level();

        public native CvTreeNodeIterator max_level(int i);

        @MemberGetter
        @Const
        public native Pointer node();

        static {
            Loader.load();
        }

        public CvTreeNodeIterator() {
            super((Pointer) null);
            allocate();
        }

        public CvTreeNodeIterator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvTreeNodeIterator(Pointer p) {
            super(p);
        }

        public CvTreeNodeIterator position(long position) {
            return (CvTreeNodeIterator) super.position(position);
        }
    }

    @Convention("CV_STDCALL")
    public static class Cv_iplCreateImageHeader extends FunctionPointer {
        private native void allocate();

        public native IplImage call(int i, int i2, int i3, @Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2, int i4, int i5, int i6, int i7, int i8, IplROI iplROI, IplImage iplImage, Pointer pointer, IplTileInfo iplTileInfo);

        static {
            Loader.load();
        }

        public Cv_iplCreateImageHeader(Pointer p) {
            super(p);
        }

        protected Cv_iplCreateImageHeader() {
            allocate();
        }
    }

    @Convention("CV_STDCALL")
    public static class Cv_iplAllocateImageData extends FunctionPointer {
        private native void allocate();

        public native void call(IplImage iplImage, int i, int i2);

        static {
            Loader.load();
        }

        public Cv_iplAllocateImageData(Pointer p) {
            super(p);
        }

        protected Cv_iplAllocateImageData() {
            allocate();
        }
    }

    @Convention("CV_STDCALL")
    public static class Cv_iplDeallocate extends FunctionPointer {
        private native void allocate();

        public native void call(IplImage iplImage, int i);

        static {
            Loader.load();
        }

        public Cv_iplDeallocate(Pointer p) {
            super(p);
        }

        protected Cv_iplDeallocate() {
            allocate();
        }
    }

    @Convention("CV_STDCALL")
    public static class Cv_iplCreateROI extends FunctionPointer {
        private native void allocate();

        public native IplROI call(int i, int i2, int i3, int i4, int i5);

        static {
            Loader.load();
        }

        public Cv_iplCreateROI(Pointer p) {
            super(p);
        }

        protected Cv_iplCreateROI() {
            allocate();
        }
    }

    @Convention("CV_STDCALL")
    public static class Cv_iplCloneImage extends FunctionPointer {
        private native void allocate();

        public native IplImage call(@Const IplImage iplImage);

        static {
            Loader.load();
        }

        public Cv_iplCloneImage(Pointer p) {
            super(p);
        }

        protected Cv_iplCloneImage() {
            allocate();
        }
    }

    @Convention("CV_CDECL")
    public static class CvErrorCallback extends FunctionPointer {
        private native void allocate();

        public native int call(int i, @Cast({"const char*"}) BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"const char*"}) BytePointer bytePointer3, int i2, Pointer pointer);

        static {
            Loader.load();
        }

        public CvErrorCallback(Pointer p) {
            super(p);
        }

        protected CvErrorCallback() {
            allocate();
        }
    }

    @Name({"cv::DefaultDeleter<CvMat>"})
    public static class CvMatDefaultDeleter extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @Name({"operator ()"})
        public native void apply(CvMat cvMat);

        static {
            Loader.load();
        }

        public CvMatDefaultDeleter() {
            super((Pointer) null);
            allocate();
        }

        public CvMatDefaultDeleter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvMatDefaultDeleter(Pointer p) {
            super(p);
        }

        public CvMatDefaultDeleter position(long position) {
            return (CvMatDefaultDeleter) super.position(position);
        }
    }

    @Name({"cv::Complex<float>"})
    @NoOffset
    public static class Complexf extends FloatPointer {
        private native void allocate();

        private native void allocate(float f);

        private native void allocate(float f, float f2);

        private native void allocateArray(long j);

        @ByVal
        public native Complexf conj();

        public native float im();

        public native Complexf im(float f);

        public native float re();

        public native Complexf re(float f);

        static {
            Loader.load();
        }

        public Complexf(Pointer p) {
            super(p);
        }

        public Complexf(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Complexf position(long position) {
            return (Complexf) super.position(position);
        }

        public Complexf() {
            super((Pointer) null);
            allocate();
        }

        public Complexf(float _re, float _im) {
            super((Pointer) null);
            allocate(_re, _im);
        }

        public Complexf(float _re) {
            super((Pointer) null);
            allocate(_re);
        }
    }

    @Name({"cv::Complex<double>"})
    @NoOffset
    public static class Complexd extends DoublePointer {
        private native void allocate();

        private native void allocate(double d);

        private native void allocate(double d, double d2);

        private native void allocateArray(long j);

        @ByVal
        public native Complexd conj();

        public native double im();

        public native Complexd im(double d);

        public native double re();

        public native Complexd re(double d);

        static {
            Loader.load();
        }

        public Complexd(Pointer p) {
            super(p);
        }

        public Complexd(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Complexd position(long position) {
            return (Complexd) super.position(position);
        }

        public Complexd() {
            super((Pointer) null);
            allocate();
        }

        public Complexd(double _re, double _im) {
            super((Pointer) null);
            allocate(_re, _im);
        }

        public Complexd(double _re) {
            super((Pointer) null);
            allocate(_re);
        }
    }

    @Name({"cv::Point_<int>"})
    @NoOffset
    public static class Point extends IntPointer {
        private native void allocate();

        private native void allocate(int i, int i2);

        private native void allocate(@ByRef @Const Point point);

        private native void allocate(@ByRef @Const Size size);

        private native void allocateArray(long j);

        public native double cross(@ByRef @Const Point point);

        public native double ddot(@ByRef @Const Point point);

        public native int dot(@ByRef @Const Point point);

        @Cast({"bool"})
        public native boolean inside(@ByRef @Const Rect rect);

        @ByRef
        @Name({"operator ="})
        public native Point put(@ByRef @Const Point point);

        public native int x();

        public native Point x(int i);

        public native int y();

        public native Point y(int i);

        static {
            Loader.load();
        }

        public Point(Pointer p) {
            super(p);
        }

        public Point(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Point position(long position) {
            return (Point) super.position(position);
        }

        public Point() {
            super((Pointer) null);
            allocate();
        }

        public Point(int _x, int _y) {
            super((Pointer) null);
            allocate(_x, _y);
        }

        public Point(@ByRef @Const Point pt) {
            super((Pointer) null);
            allocate(pt);
        }

        public Point(@ByRef @Const Size sz) {
            super((Pointer) null);
            allocate(sz);
        }
    }

    @Name({"cv::Point_<float>"})
    @NoOffset
    public static class Point2f extends FloatPointer {
        private native void allocate();

        private native void allocate(float f, float f2);

        private native void allocate(@ByRef @Const Point2f point2f);

        private native void allocate(@ByRef @Const Size2f size2f);

        private native void allocateArray(long j);

        public native double cross(@ByRef @Const Point2f point2f);

        public native double ddot(@ByRef @Const Point2f point2f);

        public native float dot(@ByRef @Const Point2f point2f);

        @Cast({"bool"})
        public native boolean inside(@ByRef @Const Rect2f rect2f);

        @ByRef
        @Name({"operator ="})
        public native Point2f put(@ByRef @Const Point2f point2f);

        public native float x();

        public native Point2f x(float f);

        public native float y();

        public native Point2f y(float f);

        static {
            Loader.load();
        }

        public Point2f(Pointer p) {
            super(p);
        }

        public Point2f(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Point2f position(long position) {
            return (Point2f) super.position(position);
        }

        public Point2f() {
            super((Pointer) null);
            allocate();
        }

        public Point2f(float _x, float _y) {
            super((Pointer) null);
            allocate(_x, _y);
        }

        public Point2f(@ByRef @Const Point2f pt) {
            super((Pointer) null);
            allocate(pt);
        }

        public Point2f(@ByRef @Const Size2f sz) {
            super((Pointer) null);
            allocate(sz);
        }
    }

    @Name({"cv::Point_<double>"})
    @NoOffset
    public static class Point2d extends DoublePointer {
        private native void allocate();

        private native void allocate(double d, double d2);

        private native void allocate(@ByRef @Const Point2d point2d);

        private native void allocate(@ByRef @Const Size2d size2d);

        private native void allocateArray(long j);

        public native double cross(@ByRef @Const Point2d point2d);

        public native double ddot(@ByRef @Const Point2d point2d);

        public native double dot(@ByRef @Const Point2d point2d);

        @Cast({"bool"})
        public native boolean inside(@ByRef @Const Rect2d rect2d);

        @ByRef
        @Name({"operator ="})
        public native Point2d put(@ByRef @Const Point2d point2d);

        public native double x();

        public native Point2d x(double d);

        public native double y();

        public native Point2d y(double d);

        static {
            Loader.load();
        }

        public Point2d(Pointer p) {
            super(p);
        }

        public Point2d(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Point2d position(long position) {
            return (Point2d) super.position(position);
        }

        public Point2d() {
            super((Pointer) null);
            allocate();
        }

        public Point2d(double _x, double _y) {
            super((Pointer) null);
            allocate(_x, _y);
        }

        public Point2d(@ByRef @Const Point2d pt) {
            super((Pointer) null);
            allocate(pt);
        }

        public Point2d(@ByRef @Const Size2d sz) {
            super((Pointer) null);
            allocate(sz);
        }
    }

    @Name({"cv::Point3_<int>"})
    @NoOffset
    public static class Point3i extends IntPointer {
        private native void allocate();

        private native void allocate(int i, int i2, int i3);

        private native void allocate(@ByRef @Const Point3i point3i);

        private native void allocate(@ByRef @Const Point point);

        private native void allocateArray(long j);

        @ByVal
        public native Point3i cross(@ByRef @Const Point3i point3i);

        public native double ddot(@ByRef @Const Point3i point3i);

        public native int dot(@ByRef @Const Point3i point3i);

        @ByRef
        @Name({"operator ="})
        public native Point3i put(@ByRef @Const Point3i point3i);

        public native int x();

        public native Point3i x(int i);

        public native int y();

        public native Point3i y(int i);

        public native int z();

        public native Point3i z(int i);

        static {
            Loader.load();
        }

        public Point3i(Pointer p) {
            super(p);
        }

        public Point3i(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Point3i position(long position) {
            return (Point3i) super.position(position);
        }

        public Point3i() {
            super((Pointer) null);
            allocate();
        }

        public Point3i(int _x, int _y, int _z) {
            super((Pointer) null);
            allocate(_x, _y, _z);
        }

        public Point3i(@ByRef @Const Point3i pt) {
            super((Pointer) null);
            allocate(pt);
        }

        public Point3i(@ByRef @Const Point pt) {
            super((Pointer) null);
            allocate(pt);
        }
    }

    @Name({"cv::Point3_<float>"})
    @NoOffset
    public static class Point3f extends FloatPointer {
        private native void allocate();

        private native void allocate(float f, float f2, float f3);

        private native void allocate(@ByRef @Const Point2f point2f);

        private native void allocate(@ByRef @Const Point3f point3f);

        private native void allocateArray(long j);

        @ByVal
        public native Point3f cross(@ByRef @Const Point3f point3f);

        public native double ddot(@ByRef @Const Point3f point3f);

        public native float dot(@ByRef @Const Point3f point3f);

        @ByRef
        @Name({"operator ="})
        public native Point3f put(@ByRef @Const Point3f point3f);

        public native float x();

        public native Point3f x(float f);

        public native float y();

        public native Point3f y(float f);

        public native float z();

        public native Point3f z(float f);

        static {
            Loader.load();
        }

        public Point3f(Pointer p) {
            super(p);
        }

        public Point3f(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Point3f position(long position) {
            return (Point3f) super.position(position);
        }

        public Point3f() {
            super((Pointer) null);
            allocate();
        }

        public Point3f(float _x, float _y, float _z) {
            super((Pointer) null);
            allocate(_x, _y, _z);
        }

        public Point3f(@ByRef @Const Point3f pt) {
            super((Pointer) null);
            allocate(pt);
        }

        public Point3f(@ByRef @Const Point2f pt) {
            super((Pointer) null);
            allocate(pt);
        }
    }

    @Name({"cv::Point3_<double>"})
    @NoOffset
    public static class Point3d extends DoublePointer {
        private native void allocate();

        private native void allocate(double d, double d2, double d3);

        private native void allocate(@ByRef @Const Point2d point2d);

        private native void allocate(@ByRef @Const Point3d point3d);

        private native void allocateArray(long j);

        @ByVal
        public native Point3d cross(@ByRef @Const Point3d point3d);

        public native double ddot(@ByRef @Const Point3d point3d);

        public native double dot(@ByRef @Const Point3d point3d);

        @ByRef
        @Name({"operator ="})
        public native Point3d put(@ByRef @Const Point3d point3d);

        public native double x();

        public native Point3d x(double d);

        public native double y();

        public native Point3d y(double d);

        public native double z();

        public native Point3d z(double d);

        static {
            Loader.load();
        }

        public Point3d(Pointer p) {
            super(p);
        }

        public Point3d(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Point3d position(long position) {
            return (Point3d) super.position(position);
        }

        public Point3d() {
            super((Pointer) null);
            allocate();
        }

        public Point3d(double _x, double _y, double _z) {
            super((Pointer) null);
            allocate(_x, _y, _z);
        }

        public Point3d(@ByRef @Const Point3d pt) {
            super((Pointer) null);
            allocate(pt);
        }

        public Point3d(@ByRef @Const Point2d pt) {
            super((Pointer) null);
            allocate(pt);
        }
    }

    @Name({"cv::Size_<int>"})
    @NoOffset
    public static class Size extends IntPointer {
        private native void allocate();

        private native void allocate(int i, int i2);

        private native void allocate(@ByRef @Const Point point);

        private native void allocate(@ByRef @Const Size size);

        private native void allocateArray(long j);

        public native int area();

        public native double aspectRatio();

        @Cast({"bool"})
        public native boolean empty();

        public native int height();

        public native Size height(int i);

        @ByRef
        @Name({"operator ="})
        public native Size put(@ByRef @Const Size size);

        public native int width();

        public native Size width(int i);

        static {
            Loader.load();
        }

        public Size(Pointer p) {
            super(p);
        }

        public Size(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Size position(long position) {
            return (Size) super.position(position);
        }

        public Size() {
            super((Pointer) null);
            allocate();
        }

        public Size(int _width, int _height) {
            super((Pointer) null);
            allocate(_width, _height);
        }

        public Size(@ByRef @Const Size sz) {
            super((Pointer) null);
            allocate(sz);
        }

        public Size(@ByRef @Const Point pt) {
            super((Pointer) null);
            allocate(pt);
        }
    }

    @Name({"cv::Size_<float>"})
    @NoOffset
    public static class Size2f extends FloatPointer {
        private native void allocate();

        private native void allocate(float f, float f2);

        private native void allocate(@ByRef @Const Point2f point2f);

        private native void allocate(@ByRef @Const Size2f size2f);

        private native void allocateArray(long j);

        public native float area();

        public native double aspectRatio();

        @Cast({"bool"})
        public native boolean empty();

        public native float height();

        public native Size2f height(float f);

        @ByRef
        @Name({"operator ="})
        public native Size2f put(@ByRef @Const Size2f size2f);

        public native float width();

        public native Size2f width(float f);

        static {
            Loader.load();
        }

        public Size2f(Pointer p) {
            super(p);
        }

        public Size2f(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Size2f position(long position) {
            return (Size2f) super.position(position);
        }

        public Size2f() {
            super((Pointer) null);
            allocate();
        }

        public Size2f(float _width, float _height) {
            super((Pointer) null);
            allocate(_width, _height);
        }

        public Size2f(@ByRef @Const Size2f sz) {
            super((Pointer) null);
            allocate(sz);
        }

        public Size2f(@ByRef @Const Point2f pt) {
            super((Pointer) null);
            allocate(pt);
        }
    }

    @Name({"cv::Size_<double>"})
    @NoOffset
    public static class Size2d extends DoublePointer {
        private native void allocate();

        private native void allocate(double d, double d2);

        private native void allocate(@ByRef @Const Point2d point2d);

        private native void allocate(@ByRef @Const Size2d size2d);

        private native void allocateArray(long j);

        public native double area();

        public native double aspectRatio();

        @Cast({"bool"})
        public native boolean empty();

        public native double height();

        public native Size2d height(double d);

        @ByRef
        @Name({"operator ="})
        public native Size2d put(@ByRef @Const Size2d size2d);

        public native double width();

        public native Size2d width(double d);

        static {
            Loader.load();
        }

        public Size2d(Pointer p) {
            super(p);
        }

        public Size2d(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Size2d position(long position) {
            return (Size2d) super.position(position);
        }

        public Size2d() {
            super((Pointer) null);
            allocate();
        }

        public Size2d(double _width, double _height) {
            super((Pointer) null);
            allocate(_width, _height);
        }

        public Size2d(@ByRef @Const Size2d sz) {
            super((Pointer) null);
            allocate(sz);
        }

        public Size2d(@ByRef @Const Point2d pt) {
            super((Pointer) null);
            allocate(pt);
        }
    }

    @Name({"cv::Rect_<int>"})
    @NoOffset
    public static class Rect extends IntPointer {
        private native void allocate();

        private native void allocate(int i, int i2, int i3, int i4);

        private native void allocate(@ByRef @Const Point point, @ByRef @Const Point point2);

        private native void allocate(@ByRef @Const Point point, @ByRef @Const Size size);

        private native void allocate(@ByRef @Const Rect rect);

        private native void allocateArray(long j);

        public native int area();

        @ByVal
        public native Point br();

        @Cast({"bool"})
        public native boolean contains(@ByRef @Const Point point);

        @Cast({"bool"})
        public native boolean empty();

        public native int height();

        public native Rect height(int i);

        @ByRef
        @Name({"operator ="})
        public native Rect put(@ByRef @Const Rect rect);

        @ByVal
        public native Size size();

        @ByVal
        public native Point tl();

        public native int width();

        public native Rect width(int i);

        public native int x();

        public native Rect x(int i);

        public native int y();

        public native Rect y(int i);

        static {
            Loader.load();
        }

        public Rect(Pointer p) {
            super(p);
        }

        public Rect(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Rect position(long position) {
            return (Rect) super.position(position);
        }

        public Rect() {
            super((Pointer) null);
            allocate();
        }

        public Rect(int _x, int _y, int _width, int _height) {
            super((Pointer) null);
            allocate(_x, _y, _width, _height);
        }

        public Rect(@ByRef @Const Rect r) {
            super((Pointer) null);
            allocate(r);
        }

        public Rect(@ByRef @Const Point org2, @ByRef @Const Size sz) {
            super((Pointer) null);
            allocate(org2, sz);
        }

        public Rect(@ByRef @Const Point pt1, @ByRef @Const Point pt2) {
            super((Pointer) null);
            allocate(pt1, pt2);
        }
    }

    @Name({"cv::Rect_<float>"})
    @NoOffset
    public static class Rect2f extends FloatPointer {
        private native void allocate();

        private native void allocate(float f, float f2, float f3, float f4);

        private native void allocate(@ByRef @Const Point2f point2f, @ByRef @Const Point2f point2f2);

        private native void allocate(@ByRef @Const Point2f point2f, @ByRef @Const Size2f size2f);

        private native void allocate(@ByRef @Const Rect2f rect2f);

        private native void allocateArray(long j);

        public native float area();

        @ByVal
        public native Point2f br();

        @Cast({"bool"})
        public native boolean contains(@ByRef @Const Point2f point2f);

        @Cast({"bool"})
        public native boolean empty();

        public native float height();

        public native Rect2f height(float f);

        @ByRef
        @Name({"operator ="})
        public native Rect2f put(@ByRef @Const Rect2f rect2f);

        @ByVal
        public native Size2f size();

        @ByVal
        public native Point2f tl();

        public native float width();

        public native Rect2f width(float f);

        public native float x();

        public native Rect2f x(float f);

        public native float y();

        public native Rect2f y(float f);

        static {
            Loader.load();
        }

        public Rect2f(Pointer p) {
            super(p);
        }

        public Rect2f(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Rect2f position(long position) {
            return (Rect2f) super.position(position);
        }

        public Rect2f() {
            super((Pointer) null);
            allocate();
        }

        public Rect2f(float _x, float _y, float _width, float _height) {
            super((Pointer) null);
            allocate(_x, _y, _width, _height);
        }

        public Rect2f(@ByRef @Const Rect2f r) {
            super((Pointer) null);
            allocate(r);
        }

        public Rect2f(@ByRef @Const Point2f org2, @ByRef @Const Size2f sz) {
            super((Pointer) null);
            allocate(org2, sz);
        }

        public Rect2f(@ByRef @Const Point2f pt1, @ByRef @Const Point2f pt2) {
            super((Pointer) null);
            allocate(pt1, pt2);
        }
    }

    @Name({"cv::Rect_<double>"})
    @NoOffset
    public static class Rect2d extends DoublePointer {
        private native void allocate();

        private native void allocate(double d, double d2, double d3, double d4);

        private native void allocate(@ByRef @Const Point2d point2d, @ByRef @Const Point2d point2d2);

        private native void allocate(@ByRef @Const Point2d point2d, @ByRef @Const Size2d size2d);

        private native void allocate(@ByRef @Const Rect2d rect2d);

        private native void allocateArray(long j);

        public native double area();

        @ByVal
        public native Point2d br();

        @Cast({"bool"})
        public native boolean contains(@ByRef @Const Point2d point2d);

        @Cast({"bool"})
        public native boolean empty();

        public native double height();

        public native Rect2d height(double d);

        @ByRef
        @Name({"operator ="})
        public native Rect2d put(@ByRef @Const Rect2d rect2d);

        @ByVal
        public native Size2d size();

        @ByVal
        public native Point2d tl();

        public native double width();

        public native Rect2d width(double d);

        public native double x();

        public native Rect2d x(double d);

        public native double y();

        public native Rect2d y(double d);

        static {
            Loader.load();
        }

        public Rect2d(Pointer p) {
            super(p);
        }

        public Rect2d(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Rect2d position(long position) {
            return (Rect2d) super.position(position);
        }

        public Rect2d() {
            super((Pointer) null);
            allocate();
        }

        public Rect2d(double _x, double _y, double _width, double _height) {
            super((Pointer) null);
            allocate(_x, _y, _width, _height);
        }

        public Rect2d(@ByRef @Const Rect2d r) {
            super((Pointer) null);
            allocate(r);
        }

        public Rect2d(@ByRef @Const Point2d org2, @ByRef @Const Size2d sz) {
            super((Pointer) null);
            allocate(org2, sz);
        }

        public Rect2d(@ByRef @Const Point2d pt1, @ByRef @Const Point2d pt2) {
            super((Pointer) null);
            allocate(pt1, pt2);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class RotatedRect extends FloatPointer {
        private native void allocate();

        private native void allocate(@ByRef @Const Point2f point2f, @ByRef @Const Point2f point2f2, @ByRef @Const Point2f point2f3);

        private native void allocate(@ByRef @Const Point2f point2f, @ByRef @Const Size2f size2f, float f);

        private native void allocateArray(long j);

        public native float angle();

        public native RotatedRect angle(float f);

        @ByVal
        public native Rect boundingRect();

        @ByVal
        public native Rect2f boundingRect2f();

        @ByRef
        public native Point2f center();

        public native RotatedRect center(Point2f point2f);

        public native void points(Point2f point2f);

        public native RotatedRect size(Size2f size2f);

        @ByRef
        public native Size2f size();

        static {
            Loader.load();
        }

        public RotatedRect(Pointer p) {
            super(p);
        }

        public RotatedRect(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public RotatedRect position(long position) {
            return (RotatedRect) super.position(position);
        }

        public RotatedRect() {
            super((Pointer) null);
            allocate();
        }

        public RotatedRect(@ByRef @Const Point2f center, @ByRef @Const Size2f size, float angle) {
            super((Pointer) null);
            allocate(center, size, angle);
        }

        public RotatedRect(@ByRef @Const Point2f point1, @ByRef @Const Point2f point2, @ByRef @Const Point2f point3) {
            super((Pointer) null);
            allocate(point1, point2, point3);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class Range extends Pointer {
        @ByVal
        public static native Range all();

        private native void allocate();

        private native void allocate(int i, int i2);

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean empty();

        public native int end();

        public native Range end(int i);

        public native int size();

        public native int start();

        public native Range start(int i);

        static {
            Loader.load();
        }

        public Range(Pointer p) {
            super(p);
        }

        public Range(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Range position(long position) {
            return (Range) super.position(position);
        }

        public Range() {
            super((Pointer) null);
            allocate();
        }

        public Range(int _start, int _end) {
            super((Pointer) null);
            allocate(_start, _end);
        }
    }

    @Name({"cv::Scalar_<double>"})
    public static class Scalar extends opencv_core.AbstractScalar {
        @ByVal
        public static native Scalar all(double d);

        private native void allocate();

        private native void allocate(double d);

        private native void allocate(double d, double d2);

        private native void allocate(double d, double d2, double d3, double d4);

        private native void allocate(@ByRef @Const Scalar scalar);

        private native void allocateArray(long j);

        @ByVal
        public native Scalar conj();

        @Cast({"bool"})
        public native boolean isReal();

        @ByVal
        public native Scalar mul(@ByRef @Const Scalar scalar);

        @ByVal
        public native Scalar mul(@ByRef @Const Scalar scalar, double d);

        @ByRef
        @Name({"operator ="})
        public native Scalar put(@ByRef @Const Scalar scalar);

        static {
            Loader.load();
        }

        public Scalar(Pointer p) {
            super(p);
        }

        public Scalar(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Scalar position(long position) {
            return (Scalar) super.position(position);
        }

        public Scalar() {
            super((Pointer) null);
            allocate();
        }

        public Scalar(double v0, double v1, double v2, double v3) {
            super((Pointer) null);
            allocate(v0, v1, v2, v3);
        }

        public Scalar(double v0, double v1) {
            super((Pointer) null);
            allocate(v0, v1);
        }

        public Scalar(double v0) {
            super((Pointer) null);
            allocate(v0);
        }

        public Scalar(@ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(s);
        }
    }

    @Name({"cv::Scalar_<int>"})
    public static class Scalar4i extends IntPointer {
        @ByVal
        public static native Scalar4i all(int i);

        private native void allocate();

        private native void allocate(int i);

        private native void allocate(int i, int i2);

        private native void allocate(int i, int i2, int i3, int i4);

        private native void allocate(@ByRef @Const Scalar4i scalar4i);

        private native void allocateArray(long j);

        @ByVal
        public native Scalar4i conj();

        @Cast({"bool"})
        public native boolean isReal();

        @ByVal
        public native Scalar4i mul(@ByRef @Const Scalar4i scalar4i);

        @ByVal
        public native Scalar4i mul(@ByRef @Const Scalar4i scalar4i, double d);

        @ByRef
        @Name({"operator ="})
        public native Scalar4i put(@ByRef @Const Scalar4i scalar4i);

        static {
            Loader.load();
        }

        public Scalar4i(Pointer p) {
            super(p);
        }

        public Scalar4i(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Scalar4i position(long position) {
            return (Scalar4i) super.position(position);
        }

        public Scalar4i() {
            super((Pointer) null);
            allocate();
        }

        public Scalar4i(int v0, int v1, int v2, int v3) {
            super((Pointer) null);
            allocate(v0, v1, v2, v3);
        }

        public Scalar4i(int v0, int v1) {
            super((Pointer) null);
            allocate(v0, v1);
        }

        public Scalar4i(int v0) {
            super((Pointer) null);
            allocate(v0);
        }

        public Scalar4i(@ByRef @Const Scalar4i s) {
            super((Pointer) null);
            allocate(s);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class KeyPoint extends Pointer {
        private native void allocate();

        private native void allocate(float f, float f2, float f3);

        private native void allocate(float f, float f2, float f3, float f4, float f5, int i, int i2);

        private native void allocate(@ByVal Point2f point2f, float f);

        private native void allocate(@ByVal Point2f point2f, float f, float f2, float f3, int i, int i2);

        private native void allocateArray(long j);

        public static native void convert(@ByRef @Const KeyPointVector keyPointVector, @ByRef Point2fVector point2fVector);

        public static native void convert(@ByRef @Const KeyPointVector keyPointVector, @ByRef Point2fVector point2fVector, @StdVector IntBuffer intBuffer);

        public static native void convert(@ByRef @Const KeyPointVector keyPointVector, @ByRef Point2fVector point2fVector, @StdVector IntPointer intPointer);

        public static native void convert(@ByRef @Const KeyPointVector keyPointVector, @ByRef Point2fVector point2fVector, @StdVector int[] iArr);

        public static native void convert(@ByRef @Const Point2fVector point2fVector, @ByRef KeyPointVector keyPointVector);

        public static native void convert(@ByRef @Const Point2fVector point2fVector, @ByRef KeyPointVector keyPointVector, float f, float f2, int i, int i2);

        public static native float overlap(@ByRef @Const KeyPoint keyPoint, @ByRef @Const KeyPoint keyPoint2);

        public native float angle();

        public native KeyPoint angle(float f);

        public native int class_id();

        public native KeyPoint class_id(int i);

        @Cast({"size_t"})
        public native long hash();

        public native int octave();

        public native KeyPoint octave(int i);

        public native KeyPoint pt(Point2f point2f);

        @ByRef
        public native Point2f pt();

        public native float response();

        public native KeyPoint response(float f);

        public native float size();

        public native KeyPoint size(float f);

        static {
            Loader.load();
        }

        public KeyPoint(Pointer p) {
            super(p);
        }

        public KeyPoint(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public KeyPoint position(long position) {
            return (KeyPoint) super.position(position);
        }

        public KeyPoint() {
            super((Pointer) null);
            allocate();
        }

        public KeyPoint(@ByVal Point2f _pt, float _size, float _angle, float _response, int _octave, int _class_id) {
            super((Pointer) null);
            allocate(_pt, _size, _angle, _response, _octave, _class_id);
        }

        public KeyPoint(@ByVal Point2f _pt, float _size) {
            super((Pointer) null);
            allocate(_pt, _size);
        }

        public KeyPoint(float x, float y, float _size, float _angle, float _response, int _octave, int _class_id) {
            super((Pointer) null);
            allocate(x, y, _size, _angle, _response, _octave, _class_id);
        }

        public KeyPoint(float x, float y, float _size) {
            super((Pointer) null);
            allocate(x, y, _size);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class DMatch extends Pointer {
        private native void allocate();

        private native void allocate(int i, int i2, float f);

        private native void allocate(int i, int i2, int i3, float f);

        private native void allocateArray(long j);

        public native float distance();

        public native DMatch distance(float f);

        public native int imgIdx();

        public native DMatch imgIdx(int i);

        @Cast({"bool"})
        @Name({"operator <"})
        public native boolean lessThan(@ByRef @Const DMatch dMatch);

        public native int queryIdx();

        public native DMatch queryIdx(int i);

        public native int trainIdx();

        public native DMatch trainIdx(int i);

        static {
            Loader.load();
        }

        public DMatch(Pointer p) {
            super(p);
        }

        public DMatch(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public DMatch position(long position) {
            return (DMatch) super.position(position);
        }

        public DMatch() {
            super((Pointer) null);
            allocate();
        }

        public DMatch(int _queryIdx, int _trainIdx, float _distance) {
            super((Pointer) null);
            allocate(_queryIdx, _trainIdx, _distance);
        }

        public DMatch(int _queryIdx, int _trainIdx, int _imgIdx, float _distance) {
            super((Pointer) null);
            allocate(_queryIdx, _trainIdx, _imgIdx, _distance);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TermCriteria extends Pointer {
        public static final int COUNT = 1;
        public static final int EPS = 2;
        public static final int MAX_ITER = 1;

        private native void allocate();

        private native void allocate(int i, int i2, double d);

        private native void allocateArray(long j);

        public native double epsilon();

        public native TermCriteria epsilon(double d);

        @Cast({"bool"})
        public native boolean isValid();

        public native int maxCount();

        public native TermCriteria maxCount(int i);

        public native int type();

        public native TermCriteria type(int i);

        static {
            Loader.load();
        }

        public TermCriteria(Pointer p) {
            super(p);
        }

        public TermCriteria(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TermCriteria position(long position) {
            return (TermCriteria) super.position(position);
        }

        public TermCriteria() {
            super((Pointer) null);
            allocate();
        }

        public TermCriteria(int type, int maxCount, double epsilon) {
            super((Pointer) null);
            allocate(type, maxCount, epsilon);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class Moments extends Pointer {
        private native void allocate();

        private native void allocate(double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, double d9, double d10);

        private native void allocateArray(long j);

        public native double m00();

        public native Moments m00(double d);

        public native double m01();

        public native Moments m01(double d);

        public native double m02();

        public native Moments m02(double d);

        public native double m03();

        public native Moments m03(double d);

        public native double m10();

        public native Moments m10(double d);

        public native double m11();

        public native Moments m11(double d);

        public native double m12();

        public native Moments m12(double d);

        public native double m20();

        public native Moments m20(double d);

        public native double m21();

        public native Moments m21(double d);

        public native double m30();

        public native Moments m30(double d);

        public native double mu02();

        public native Moments mu02(double d);

        public native double mu03();

        public native Moments mu03(double d);

        public native double mu11();

        public native Moments mu11(double d);

        public native double mu12();

        public native Moments mu12(double d);

        public native double mu20();

        public native Moments mu20(double d);

        public native double mu21();

        public native Moments mu21(double d);

        public native double mu30();

        public native Moments mu30(double d);

        public native double nu02();

        public native Moments nu02(double d);

        public native double nu03();

        public native Moments nu03(double d);

        public native double nu11();

        public native Moments nu11(double d);

        public native double nu12();

        public native Moments nu12(double d);

        public native double nu20();

        public native Moments nu20(double d);

        public native double nu21();

        public native Moments nu21(double d);

        public native double nu30();

        public native Moments nu30(double d);

        static {
            Loader.load();
        }

        public Moments(Pointer p) {
            super(p);
        }

        public Moments(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Moments position(long position) {
            return (Moments) super.position(position);
        }

        public Moments() {
            super((Pointer) null);
            allocate();
        }

        public Moments(double m00, double m10, double m01, double m20, double m11, double m02, double m30, double m21, double m12, double m03) {
            super((Pointer) null);
            allocate(m00, m10, m01, m20, m11, m02, m30, m21, m12, m03);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class PCA extends Pointer {
        public static final int DATA_AS_COL = 1;
        public static final int DATA_AS_ROW = 0;
        public static final int USE_AVG = 2;

        private native void allocate();

        private native void allocate(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

        private native void allocate(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, double d);

        private native void allocate(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, int i2);

        private native void allocate(@ByVal Mat mat, @ByVal Mat mat2, int i);

        private native void allocate(@ByVal Mat mat, @ByVal Mat mat2, int i, double d);

        private native void allocate(@ByVal Mat mat, @ByVal Mat mat2, int i, int i2);

        private native void allocate(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

        private native void allocate(@ByVal UMat uMat, @ByVal UMat uMat2, int i, double d);

        private native void allocate(@ByVal UMat uMat, @ByVal UMat uMat2, int i, int i2);

        private native void allocateArray(long j);

        @ByRef
        @Name({"operator ()"})
        public native PCA apply(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

        @ByRef
        @Name({"operator ()"})
        public native PCA apply(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, double d);

        @ByRef
        @Name({"operator ()"})
        public native PCA apply(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i, int i2);

        @ByRef
        @Name({"operator ()"})
        public native PCA apply(@ByVal Mat mat, @ByVal Mat mat2, int i);

        @ByRef
        @Name({"operator ()"})
        public native PCA apply(@ByVal Mat mat, @ByVal Mat mat2, int i, double d);

        @ByRef
        @Name({"operator ()"})
        public native PCA apply(@ByVal Mat mat, @ByVal Mat mat2, int i, int i2);

        @ByRef
        @Name({"operator ()"})
        public native PCA apply(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

        @ByRef
        @Name({"operator ()"})
        public native PCA apply(@ByVal UMat uMat, @ByVal UMat uMat2, int i, double d);

        @ByRef
        @Name({"operator ()"})
        public native PCA apply(@ByVal UMat uMat, @ByVal UMat uMat2, int i, int i2);

        @ByVal
        public native Mat backProject(@ByVal GpuMat gpuMat);

        @ByVal
        public native Mat backProject(@ByVal Mat mat);

        @ByVal
        public native Mat backProject(@ByVal UMat uMat);

        public native void backProject(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

        public native void backProject(@ByVal Mat mat, @ByVal Mat mat2);

        public native void backProject(@ByVal UMat uMat, @ByVal UMat uMat2);

        @ByRef
        public native Mat eigenvalues();

        public native PCA eigenvalues(Mat mat);

        @ByRef
        public native Mat eigenvectors();

        public native PCA eigenvectors(Mat mat);

        @ByRef
        public native Mat mean();

        public native PCA mean(Mat mat);

        @ByVal
        public native Mat project(@ByVal GpuMat gpuMat);

        @ByVal
        public native Mat project(@ByVal Mat mat);

        @ByVal
        public native Mat project(@ByVal UMat uMat);

        public native void project(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

        public native void project(@ByVal Mat mat, @ByVal Mat mat2);

        public native void project(@ByVal UMat uMat, @ByVal UMat uMat2);

        public native void read(@ByRef @Const FileNode fileNode);

        public native void write(@ByRef FileStorage fileStorage);

        static {
            Loader.load();
        }

        public PCA(Pointer p) {
            super(p);
        }

        public PCA(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public PCA position(long position) {
            return (PCA) super.position(position);
        }

        public PCA() {
            super((Pointer) null);
            allocate();
        }

        public PCA(@ByVal Mat data, @ByVal Mat mean, int flags, int maxComponents) {
            super((Pointer) null);
            allocate(data, mean, flags, maxComponents);
        }

        public PCA(@ByVal Mat data, @ByVal Mat mean, int flags) {
            super((Pointer) null);
            allocate(data, mean, flags);
        }

        public PCA(@ByVal UMat data, @ByVal UMat mean, int flags, int maxComponents) {
            super((Pointer) null);
            allocate(data, mean, flags, maxComponents);
        }

        public PCA(@ByVal UMat data, @ByVal UMat mean, int flags) {
            super((Pointer) null);
            allocate(data, mean, flags);
        }

        public PCA(@ByVal GpuMat data, @ByVal GpuMat mean, int flags, int maxComponents) {
            super((Pointer) null);
            allocate(data, mean, flags, maxComponents);
        }

        public PCA(@ByVal GpuMat data, @ByVal GpuMat mean, int flags) {
            super((Pointer) null);
            allocate(data, mean, flags);
        }

        public PCA(@ByVal Mat data, @ByVal Mat mean, int flags, double retainedVariance) {
            super((Pointer) null);
            allocate(data, mean, flags, retainedVariance);
        }

        public PCA(@ByVal UMat data, @ByVal UMat mean, int flags, double retainedVariance) {
            super((Pointer) null);
            allocate(data, mean, flags, retainedVariance);
        }

        public PCA(@ByVal GpuMat data, @ByVal GpuMat mean, int flags, double retainedVariance) {
            super((Pointer) null);
            allocate(data, mean, flags, retainedVariance);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class LDA extends Pointer {
        private native void allocate();

        private native void allocate(int i);

        private native void allocate(@ByVal GpuMatVector gpuMatVector, @ByVal GpuMat gpuMat);

        private native void allocate(@ByVal GpuMatVector gpuMatVector, @ByVal GpuMat gpuMat, int i);

        private native void allocate(@ByVal GpuMatVector gpuMatVector, @ByVal Mat mat);

        private native void allocate(@ByVal GpuMatVector gpuMatVector, @ByVal Mat mat, int i);

        private native void allocate(@ByVal GpuMatVector gpuMatVector, @ByVal UMat uMat);

        private native void allocate(@ByVal GpuMatVector gpuMatVector, @ByVal UMat uMat, int i);

        private native void allocate(@ByVal MatVector matVector, @ByVal GpuMat gpuMat);

        private native void allocate(@ByVal MatVector matVector, @ByVal GpuMat gpuMat, int i);

        private native void allocate(@ByVal MatVector matVector, @ByVal Mat mat);

        private native void allocate(@ByVal MatVector matVector, @ByVal Mat mat, int i);

        private native void allocate(@ByVal MatVector matVector, @ByVal UMat uMat);

        private native void allocate(@ByVal MatVector matVector, @ByVal UMat uMat, int i);

        private native void allocate(@ByVal UMatVector uMatVector, @ByVal GpuMat gpuMat);

        private native void allocate(@ByVal UMatVector uMatVector, @ByVal GpuMat gpuMat, int i);

        private native void allocate(@ByVal UMatVector uMatVector, @ByVal Mat mat);

        private native void allocate(@ByVal UMatVector uMatVector, @ByVal Mat mat, int i);

        private native void allocate(@ByVal UMatVector uMatVector, @ByVal UMat uMat);

        private native void allocate(@ByVal UMatVector uMatVector, @ByVal UMat uMat, int i);

        private native void allocateArray(long j);

        @ByVal
        public static native Mat subspaceProject(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

        @ByVal
        public static native Mat subspaceProject(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

        @ByVal
        public static native Mat subspaceProject(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

        @ByVal
        public static native Mat subspaceReconstruct(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

        @ByVal
        public static native Mat subspaceReconstruct(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3);

        @ByVal
        public static native Mat subspaceReconstruct(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3);

        public native void compute(@ByVal GpuMatVector gpuMatVector, @ByVal GpuMat gpuMat);

        public native void compute(@ByVal GpuMatVector gpuMatVector, @ByVal Mat mat);

        public native void compute(@ByVal GpuMatVector gpuMatVector, @ByVal UMat uMat);

        public native void compute(@ByVal MatVector matVector, @ByVal GpuMat gpuMat);

        public native void compute(@ByVal MatVector matVector, @ByVal Mat mat);

        public native void compute(@ByVal MatVector matVector, @ByVal UMat uMat);

        public native void compute(@ByVal UMatVector uMatVector, @ByVal GpuMat gpuMat);

        public native void compute(@ByVal UMatVector uMatVector, @ByVal Mat mat);

        public native void compute(@ByVal UMatVector uMatVector, @ByVal UMat uMat);

        @ByVal
        public native Mat eigenvalues();

        @ByVal
        public native Mat eigenvectors();

        public native void load(@opencv_core.Str String str);

        public native void load(@opencv_core.Str BytePointer bytePointer);

        public native void load(@ByRef @Const FileStorage fileStorage);

        @ByVal
        public native Mat project(@ByVal GpuMat gpuMat);

        @ByVal
        public native Mat project(@ByVal Mat mat);

        @ByVal
        public native Mat project(@ByVal UMat uMat);

        @ByVal
        public native Mat reconstruct(@ByVal GpuMat gpuMat);

        @ByVal
        public native Mat reconstruct(@ByVal Mat mat);

        @ByVal
        public native Mat reconstruct(@ByVal UMat uMat);

        public native void save(@opencv_core.Str String str);

        public native void save(@opencv_core.Str BytePointer bytePointer);

        public native void save(@ByRef FileStorage fileStorage);

        static {
            Loader.load();
        }

        public LDA(Pointer p) {
            super(p);
        }

        public LDA(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public LDA position(long position) {
            return (LDA) super.position(position);
        }

        public LDA(int num_components) {
            super((Pointer) null);
            allocate(num_components);
        }

        public LDA() {
            super((Pointer) null);
            allocate();
        }

        public LDA(@ByVal MatVector src, @ByVal Mat labels, int num_components) {
            super((Pointer) null);
            allocate(src, labels, num_components);
        }

        public LDA(@ByVal MatVector src, @ByVal Mat labels) {
            super((Pointer) null);
            allocate(src, labels);
        }

        public LDA(@ByVal UMatVector src, @ByVal Mat labels, int num_components) {
            super((Pointer) null);
            allocate(src, labels, num_components);
        }

        public LDA(@ByVal UMatVector src, @ByVal Mat labels) {
            super((Pointer) null);
            allocate(src, labels);
        }

        public LDA(@ByVal GpuMatVector src, @ByVal Mat labels, int num_components) {
            super((Pointer) null);
            allocate(src, labels, num_components);
        }

        public LDA(@ByVal GpuMatVector src, @ByVal Mat labels) {
            super((Pointer) null);
            allocate(src, labels);
        }

        public LDA(@ByVal MatVector src, @ByVal UMat labels, int num_components) {
            super((Pointer) null);
            allocate(src, labels, num_components);
        }

        public LDA(@ByVal MatVector src, @ByVal UMat labels) {
            super((Pointer) null);
            allocate(src, labels);
        }

        public LDA(@ByVal UMatVector src, @ByVal UMat labels, int num_components) {
            super((Pointer) null);
            allocate(src, labels, num_components);
        }

        public LDA(@ByVal UMatVector src, @ByVal UMat labels) {
            super((Pointer) null);
            allocate(src, labels);
        }

        public LDA(@ByVal GpuMatVector src, @ByVal UMat labels, int num_components) {
            super((Pointer) null);
            allocate(src, labels, num_components);
        }

        public LDA(@ByVal GpuMatVector src, @ByVal UMat labels) {
            super((Pointer) null);
            allocate(src, labels);
        }

        public LDA(@ByVal MatVector src, @ByVal GpuMat labels, int num_components) {
            super((Pointer) null);
            allocate(src, labels, num_components);
        }

        public LDA(@ByVal MatVector src, @ByVal GpuMat labels) {
            super((Pointer) null);
            allocate(src, labels);
        }

        public LDA(@ByVal UMatVector src, @ByVal GpuMat labels, int num_components) {
            super((Pointer) null);
            allocate(src, labels, num_components);
        }

        public LDA(@ByVal UMatVector src, @ByVal GpuMat labels) {
            super((Pointer) null);
            allocate(src, labels);
        }

        public LDA(@ByVal GpuMatVector src, @ByVal GpuMat labels, int num_components) {
            super((Pointer) null);
            allocate(src, labels, num_components);
        }

        public LDA(@ByVal GpuMatVector src, @ByVal GpuMat labels) {
            super((Pointer) null);
            allocate(src, labels);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class SVD extends Pointer {
        public static final int FULL_UV = 4;
        public static final int MODIFY_A = 1;
        public static final int NO_UV = 2;

        private native void allocate();

        private native void allocate(@ByVal GpuMat gpuMat);

        private native void allocate(@ByVal GpuMat gpuMat, int i);

        private native void allocate(@ByVal Mat mat);

        private native void allocate(@ByVal Mat mat, int i);

        private native void allocate(@ByVal UMat uMat);

        private native void allocate(@ByVal UMat uMat, int i);

        private native void allocateArray(long j);

        public static native void backSubst(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4, @ByVal GpuMat gpuMat5);

        public static native void backSubst(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4, @ByVal Mat mat5);

        public static native void backSubst(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4, @ByVal UMat uMat5);

        public static native void compute(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

        public static native void compute(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, int i);

        public static native void compute(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4);

        public static native void compute(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @ByVal GpuMat gpuMat4, int i);

        public static native void compute(@ByVal Mat mat, @ByVal Mat mat2);

        public static native void compute(@ByVal Mat mat, @ByVal Mat mat2, int i);

        public static native void compute(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4);

        public static native void compute(@ByVal Mat mat, @ByVal Mat mat2, @ByVal Mat mat3, @ByVal Mat mat4, int i);

        public static native void compute(@ByVal UMat uMat, @ByVal UMat uMat2);

        public static native void compute(@ByVal UMat uMat, @ByVal UMat uMat2, int i);

        public static native void compute(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4);

        public static native void compute(@ByVal UMat uMat, @ByVal UMat uMat2, @ByVal UMat uMat3, @ByVal UMat uMat4, int i);

        public static native void solveZ(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

        public static native void solveZ(@ByVal Mat mat, @ByVal Mat mat2);

        public static native void solveZ(@ByVal UMat uMat, @ByVal UMat uMat2);

        @ByRef
        @Name({"operator ()"})
        public native SVD apply(@ByVal GpuMat gpuMat);

        @ByRef
        @Name({"operator ()"})
        public native SVD apply(@ByVal GpuMat gpuMat, int i);

        @ByRef
        @Name({"operator ()"})
        public native SVD apply(@ByVal Mat mat);

        @ByRef
        @Name({"operator ()"})
        public native SVD apply(@ByVal Mat mat, int i);

        @ByRef
        @Name({"operator ()"})
        public native SVD apply(@ByVal UMat uMat);

        @ByRef
        @Name({"operator ()"})
        public native SVD apply(@ByVal UMat uMat, int i);

        public native void backSubst(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

        public native void backSubst(@ByVal Mat mat, @ByVal Mat mat2);

        public native void backSubst(@ByVal UMat uMat, @ByVal UMat uMat2);

        @ByRef
        public native Mat u();

        public native SVD u(Mat mat);

        @ByRef
        public native Mat vt();

        public native SVD vt(Mat mat);

        @ByRef
        public native Mat w();

        public native SVD w(Mat mat);

        static {
            Loader.load();
        }

        public SVD(Pointer p) {
            super(p);
        }

        public SVD(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public SVD position(long position) {
            return (SVD) super.position(position);
        }

        public SVD() {
            super((Pointer) null);
            allocate();
        }

        public SVD(@ByVal Mat src, int flags) {
            super((Pointer) null);
            allocate(src, flags);
        }

        public SVD(@ByVal Mat src) {
            super((Pointer) null);
            allocate(src);
        }

        public SVD(@ByVal UMat src, int flags) {
            super((Pointer) null);
            allocate(src, flags);
        }

        public SVD(@ByVal UMat src) {
            super((Pointer) null);
            allocate(src);
        }

        public SVD(@ByVal GpuMat src, int flags) {
            super((Pointer) null);
            allocate(src, flags);
        }

        public SVD(@ByVal GpuMat src) {
            super((Pointer) null);
            allocate(src);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class RNG extends Pointer {
        public static final int NORMAL = 1;
        public static final int UNIFORM = 0;

        private native void allocate();

        private native void allocate(@Cast({"uint64"}) int i);

        private native void allocateArray(long j);

        @Name({"fill"})
        public native void _fill(@ByVal GpuMat gpuMat, int i, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3);

        @Name({"fill"})
        public native void _fill(@ByVal GpuMat gpuMat, int i, @ByVal GpuMat gpuMat2, @ByVal GpuMat gpuMat3, @Cast({"bool"}) boolean z);

        @Name({"fill"})
        public native void _fill(@ByVal Mat mat, int i, @ByVal Mat mat2, @ByVal Mat mat3);

        @Name({"fill"})
        public native void _fill(@ByVal Mat mat, int i, @ByVal Mat mat2, @ByVal Mat mat3, @Cast({"bool"}) boolean z);

        @Name({"fill"})
        public native void _fill(@ByVal UMat uMat, int i, @ByVal UMat uMat2, @ByVal UMat uMat3);

        @Name({"fill"})
        public native void _fill(@ByVal UMat uMat, int i, @ByVal UMat uMat2, @ByVal UMat uMat3, @Cast({"bool"}) boolean z);

        @Cast({"unsigned"})
        @Name({"operator ()"})
        public native int apply();

        @Cast({"unsigned"})
        @Name({"operator ()"})
        public native int apply(@Cast({"unsigned"}) int i);

        @Cast({"uchar"})
        @Name({"operator uchar"})
        public native byte asByte();

        @Name({"operator double"})
        public native double asDouble();

        @Name({"operator float"})
        public native float asFloat();

        @Cast({"unsigned"})
        @Name({"operator unsigned"})
        public native int asInt();

        @Cast({"ushort"})
        @Name({"operator ushort"})
        public native short asShort();

        @Cast({"bool"})
        @Name({"operator =="})
        public native boolean equals(@ByRef @Const RNG rng);

        public native double gaussian(double d);

        @Cast({"unsigned"})
        public native int next();

        @Cast({"uint64"})
        public native int state();

        public native RNG state(int i);

        public native double uniform(double d, double d2);

        public native float uniform(float f, float f2);

        public native int uniform(int i, int i2);

        static {
            Loader.load();
        }

        public RNG(Pointer p) {
            super(p);
        }

        public RNG(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public RNG position(long position) {
            return (RNG) super.position(position);
        }

        public RNG() {
            super((Pointer) null);
            allocate();
        }

        public RNG(@Cast({"uint64"}) int state) {
            super((Pointer) null);
            allocate(state);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class RNG_MT19937 extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"unsigned"}) int i);

        private native void allocateArray(long j);

        @Cast({"unsigned"})
        @Name({"operator ()"})
        public native int apply();

        @Cast({"unsigned"})
        @Name({"operator ()"})
        public native int apply(@Cast({"unsigned"}) int i);

        @Name({"operator double"})
        public native double asDouble();

        @Name({"operator float"})
        public native float asFloat();

        @Name({"operator int"})
        public native int asInt();

        @Cast({"unsigned"})
        public native int next();

        public native void seed(@Cast({"unsigned"}) int i);

        public native double uniform(double d, double d2);

        public native float uniform(float f, float f2);

        public native int uniform(int i, int i2);

        static {
            Loader.load();
        }

        public RNG_MT19937(Pointer p) {
            super(p);
        }

        public RNG_MT19937(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public RNG_MT19937 position(long position) {
            return (RNG_MT19937) super.position(position);
        }

        public RNG_MT19937() {
            super((Pointer) null);
            allocate();
        }

        public RNG_MT19937(@Cast({"unsigned"}) int s) {
            super((Pointer) null);
            allocate(s);
        }
    }

    @Namespace("cv")
    public static class Formatted extends Pointer {
        @Cast({"const char*"})
        public native BytePointer next();

        public native void reset();

        static {
            Loader.load();
        }

        public Formatted(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class Formatter extends Pointer {
        public static final int FMT_C = 5;
        public static final int FMT_CSV = 2;
        public static final int FMT_DEFAULT = 0;
        public static final int FMT_MATLAB = 1;
        public static final int FMT_NUMPY = 4;
        public static final int FMT_PYTHON = 3;

        @opencv_core.Ptr
        public static native Formatter get();

        @opencv_core.Ptr
        public static native Formatter get(@Cast({"cv::Formatter::FormatType"}) int i);

        @opencv_core.Ptr
        public native Formatted format(@ByRef @Const Mat mat);

        public native void set16fPrecision();

        public native void set16fPrecision(int i);

        public native void set32fPrecision();

        public native void set32fPrecision(int i);

        public native void set64fPrecision();

        public native void set64fPrecision(int i);

        public native void setMultiline();

        public native void setMultiline(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public Formatter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class Algorithm extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native void clear();

        @Cast({"bool"})
        public native boolean empty();

        @opencv_core.Str
        public native BytePointer getDefaultName();

        public native void read(@ByRef @Const FileNode fileNode);

        public native void save(@opencv_core.Str String str);

        public native void save(@opencv_core.Str BytePointer bytePointer);

        public native void write(@ByRef FileStorage fileStorage);

        public native void write(@opencv_core.Ptr FileStorage fileStorage, @opencv_core.Str String str);

        public native void write(@opencv_core.Ptr FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public Algorithm(Pointer p) {
            super(p);
        }

        public Algorithm(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Algorithm position(long position) {
            return (Algorithm) super.position(position);
        }

        public Algorithm() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::cuda")
    @NoOffset
    public static class GpuMat extends Pointer {
        private native void allocate();

        private native void allocate(int i, int i2, int i3);

        private native void allocate(int i, int i2, int i3, Pointer pointer);

        private native void allocate(int i, int i2, int i3, Pointer pointer, @Cast({"size_t"}) long j);

        private native void allocate(int i, int i2, int i3, Allocator allocator);

        private native void allocate(int i, int i2, int i3, @ByVal Scalar scalar);

        private native void allocate(int i, int i2, int i3, @ByVal Scalar scalar, Allocator allocator);

        private native void allocate(Allocator allocator);

        private native void allocate(@ByRef @Const GpuMat gpuMat);

        private native void allocate(@ByVal GpuMat gpuMat, Allocator allocator);

        private native void allocate(@ByRef @Const GpuMat gpuMat, @ByVal Range range, @ByVal Range range2);

        private native void allocate(@ByRef @Const GpuMat gpuMat, @ByVal Rect rect);

        private native void allocate(@ByVal Mat mat);

        private native void allocate(@ByVal Mat mat, Allocator allocator);

        private native void allocate(@ByVal Size size, int i);

        private native void allocate(@ByVal Size size, int i, Pointer pointer);

        private native void allocate(@ByVal Size size, int i, Pointer pointer, @Cast({"size_t"}) long j);

        private native void allocate(@ByVal Size size, int i, Allocator allocator);

        private native void allocate(@ByVal Size size, int i, @ByVal Scalar scalar);

        private native void allocate(@ByVal Size size, int i, @ByVal Scalar scalar, Allocator allocator);

        private native void allocate(@ByVal UMat uMat);

        private native void allocate(@ByVal UMat uMat, Allocator allocator);

        private native void allocateArray(long j);

        public static native Allocator defaultAllocator();

        public static native void setDefaultAllocator(Allocator allocator);

        @ByRef
        public native GpuMat adjustROI(int i, int i2, int i3, int i4);

        public native Allocator allocator();

        public native GpuMat allocator(Allocator allocator);

        @ByVal
        @Name({"operator ()"})
        public native GpuMat apply(@ByVal Range range, @ByVal Range range2);

        @ByVal
        @Name({"operator ()"})
        public native GpuMat apply(@ByVal Rect rect);

        public native void assignTo(@ByRef GpuMat gpuMat);

        public native void assignTo(@ByRef GpuMat gpuMat, int i);

        public native int channels();

        @ByVal
        public native GpuMat clone();

        @ByVal
        public native GpuMat col(int i);

        @ByVal
        public native GpuMat colRange(int i, int i2);

        @ByVal
        public native GpuMat colRange(@ByVal Range range);

        public native int cols();

        public native GpuMat cols(int i);

        public native void convertTo(@ByVal GpuMat gpuMat, int i);

        public native void convertTo(@ByVal GpuMat gpuMat, int i, double d);

        public native void convertTo(@ByVal GpuMat gpuMat, int i, double d, double d2);

        public native void convertTo(@ByVal GpuMat gpuMat, int i, double d, double d2, @ByRef Stream stream);

        public native void convertTo(@ByVal GpuMat gpuMat, int i, double d, @ByRef Stream stream);

        public native void convertTo(@ByVal GpuMat gpuMat, int i, @ByRef Stream stream);

        public native void convertTo(@ByVal Mat mat, int i);

        public native void convertTo(@ByVal Mat mat, int i, double d);

        public native void convertTo(@ByVal Mat mat, int i, double d, double d2);

        public native void convertTo(@ByVal Mat mat, int i, double d, double d2, @ByRef Stream stream);

        public native void convertTo(@ByVal Mat mat, int i, double d, @ByRef Stream stream);

        public native void convertTo(@ByVal Mat mat, int i, @ByRef Stream stream);

        public native void convertTo(@ByVal UMat uMat, int i);

        public native void convertTo(@ByVal UMat uMat, int i, double d);

        public native void convertTo(@ByVal UMat uMat, int i, double d, double d2);

        public native void convertTo(@ByVal UMat uMat, int i, double d, double d2, @ByRef Stream stream);

        public native void convertTo(@ByVal UMat uMat, int i, double d, @ByRef Stream stream);

        public native void convertTo(@ByVal UMat uMat, int i, @ByRef Stream stream);

        public native void copyTo(@ByVal GpuMat gpuMat);

        public native void copyTo(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

        public native void copyTo(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2, @ByRef Stream stream);

        public native void copyTo(@ByVal GpuMat gpuMat, @ByRef Stream stream);

        public native void copyTo(@ByVal Mat mat);

        public native void copyTo(@ByVal Mat mat, @ByVal Mat mat2);

        public native void copyTo(@ByVal Mat mat, @ByVal Mat mat2, @ByRef Stream stream);

        public native void copyTo(@ByVal Mat mat, @ByRef Stream stream);

        public native void copyTo(@ByVal UMat uMat);

        public native void copyTo(@ByVal UMat uMat, @ByRef Stream stream);

        public native void copyTo(@ByVal UMat uMat, @ByVal UMat uMat2);

        public native void copyTo(@ByVal UMat uMat, @ByVal UMat uMat2, @ByRef Stream stream);

        public native void create(int i, int i2, int i3);

        public native void create(@ByVal Size size, int i);

        @Cast({"uchar*"})
        public native BytePointer data();

        public native GpuMat data(BytePointer bytePointer);

        @MemberGetter
        @Cast({"const uchar*"})
        public native BytePointer dataend();

        @Cast({"uchar*"})
        public native BytePointer datastart();

        public native GpuMat datastart(BytePointer bytePointer);

        public native int depth();

        public native void download(@ByVal GpuMat gpuMat);

        public native void download(@ByVal GpuMat gpuMat, @ByRef Stream stream);

        public native void download(@ByVal Mat mat);

        public native void download(@ByVal Mat mat, @ByRef Stream stream);

        public native void download(@ByVal UMat uMat);

        public native void download(@ByVal UMat uMat, @ByRef Stream stream);

        @Cast({"size_t"})
        public native long elemSize();

        @Cast({"size_t"})
        public native long elemSize1();

        @Cast({"bool"})
        public native boolean empty();

        public native int flags();

        public native GpuMat flags(int i);

        @Cast({"bool"})
        public native boolean isContinuous();

        public native void locateROI(@ByRef Size size, @ByRef Point point);

        @Cast({"uchar*"})
        public native BytePointer ptr();

        @Cast({"uchar*"})
        public native BytePointer ptr(int i);

        @ByRef
        @Name({"operator ="})
        public native GpuMat put(@ByRef @Const GpuMat gpuMat);

        public native IntPointer refcount();

        public native GpuMat refcount(IntPointer intPointer);

        public native void release();

        @ByVal
        public native GpuMat reshape(int i);

        @ByVal
        public native GpuMat reshape(int i, int i2);

        @ByVal
        public native GpuMat row(int i);

        @ByVal
        public native GpuMat rowRange(int i, int i2);

        @ByVal
        public native GpuMat rowRange(@ByVal Range range);

        public native int rows();

        public native GpuMat rows(int i);

        @ByRef
        public native GpuMat setTo(@ByVal Scalar scalar);

        @ByRef
        public native GpuMat setTo(@ByVal Scalar scalar, @ByVal GpuMat gpuMat);

        @ByRef
        public native GpuMat setTo(@ByVal Scalar scalar, @ByVal GpuMat gpuMat, @ByRef Stream stream);

        @ByRef
        public native GpuMat setTo(@ByVal Scalar scalar, @ByVal Mat mat);

        @ByRef
        public native GpuMat setTo(@ByVal Scalar scalar, @ByVal Mat mat, @ByRef Stream stream);

        @ByRef
        public native GpuMat setTo(@ByVal Scalar scalar, @ByRef Stream stream);

        @ByRef
        public native GpuMat setTo(@ByVal Scalar scalar, @ByVal UMat uMat);

        @ByRef
        public native GpuMat setTo(@ByVal Scalar scalar, @ByVal UMat uMat, @ByRef Stream stream);

        @ByVal
        public native Size size();

        @Cast({"size_t"})
        public native long step();

        public native GpuMat step(long j);

        @Cast({"size_t"})
        public native long step1();

        public native void swap(@ByRef GpuMat gpuMat);

        public native int type();

        public native void updateContinuityFlag();

        public native void upload(@ByVal GpuMat gpuMat);

        public native void upload(@ByVal GpuMat gpuMat, @ByRef Stream stream);

        public native void upload(@ByVal Mat mat);

        public native void upload(@ByVal Mat mat, @ByRef Stream stream);

        public native void upload(@ByVal UMat uMat);

        public native void upload(@ByVal UMat uMat, @ByRef Stream stream);

        static {
            Loader.load();
        }

        public GpuMat(Pointer p) {
            super(p);
        }

        public GpuMat(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public GpuMat position(long position) {
            return (GpuMat) super.position(position);
        }

        public static class Allocator extends Pointer {
            @Cast({"bool"})
            @Name({"allocate"})
            public native boolean _allocate(GpuMat gpuMat, int i, int i2, @Cast({"size_t"}) long j);

            public native void free(GpuMat gpuMat);

            static {
                Loader.load();
            }

            public Allocator(Pointer p) {
                super(p);
            }
        }

        public GpuMat(Allocator allocator) {
            super((Pointer) null);
            allocate(allocator);
        }

        public GpuMat() {
            super((Pointer) null);
            allocate();
        }

        public GpuMat(int rows, int cols, int type, Allocator allocator) {
            super((Pointer) null);
            allocate(rows, cols, type, allocator);
        }

        public GpuMat(int rows, int cols, int type) {
            super((Pointer) null);
            allocate(rows, cols, type);
        }

        public GpuMat(@ByVal Size size, int type, Allocator allocator) {
            super((Pointer) null);
            allocate(size, type, allocator);
        }

        public GpuMat(@ByVal Size size, int type) {
            super((Pointer) null);
            allocate(size, type);
        }

        public GpuMat(int rows, int cols, int type, @ByVal Scalar s, Allocator allocator) {
            super((Pointer) null);
            allocate(rows, cols, type, s, allocator);
        }

        public GpuMat(int rows, int cols, int type, @ByVal Scalar s) {
            super((Pointer) null);
            allocate(rows, cols, type, s);
        }

        public GpuMat(@ByVal Size size, int type, @ByVal Scalar s, Allocator allocator) {
            super((Pointer) null);
            allocate(size, type, s, allocator);
        }

        public GpuMat(@ByVal Size size, int type, @ByVal Scalar s) {
            super((Pointer) null);
            allocate(size, type, s);
        }

        public GpuMat(@ByRef @Const GpuMat m) {
            super((Pointer) null);
            allocate(m);
        }

        public GpuMat(int rows, int cols, int type, Pointer data, @Cast({"size_t"}) long step) {
            super((Pointer) null);
            allocate(rows, cols, type, data, step);
        }

        public GpuMat(int rows, int cols, int type, Pointer data) {
            super((Pointer) null);
            allocate(rows, cols, type, data);
        }

        public GpuMat(@ByVal Size size, int type, Pointer data, @Cast({"size_t"}) long step) {
            super((Pointer) null);
            allocate(size, type, data, step);
        }

        public GpuMat(@ByVal Size size, int type, Pointer data) {
            super((Pointer) null);
            allocate(size, type, data);
        }

        public GpuMat(@ByRef @Const GpuMat m, @ByVal Range rowRange, @ByVal Range colRange) {
            super((Pointer) null);
            allocate(m, rowRange, colRange);
        }

        public GpuMat(@ByRef @Const GpuMat m, @ByVal Rect roi) {
            super((Pointer) null);
            allocate(m, roi);
        }

        public GpuMat(@ByVal Mat arr, Allocator allocator) {
            super((Pointer) null);
            allocate(arr, allocator);
        }

        public GpuMat(@ByVal Mat arr) {
            super((Pointer) null);
            allocate(arr);
        }

        public GpuMat(@ByVal UMat arr, Allocator allocator) {
            super((Pointer) null);
            allocate(arr, allocator);
        }

        public GpuMat(@ByVal UMat arr) {
            super((Pointer) null);
            allocate(arr);
        }

        public GpuMat(@ByVal GpuMat arr, Allocator allocator) {
            super((Pointer) null);
            allocate(arr, allocator);
        }
    }

    @Namespace("cv::cuda")
    @NoOffset
    public static class BufferPool extends Pointer {
        private native void allocate(@ByRef Stream stream);

        @opencv_core.Ptr
        public native GpuMat.Allocator getAllocator();

        @ByVal
        public native GpuMat getBuffer(int i, int i2, int i3);

        @ByVal
        public native GpuMat getBuffer(@ByVal Size size, int i);

        static {
            Loader.load();
        }

        public BufferPool(Pointer p) {
            super(p);
        }

        public BufferPool(@ByRef Stream stream) {
            super((Pointer) null);
            allocate(stream);
        }
    }

    @Namespace("cv::cuda")
    @NoOffset
    public static class HostMem extends Pointer {
        public static final int PAGE_LOCKED = 1;
        public static final int SHARED = 2;
        public static final int WRITE_COMBINED = 4;

        private native void allocate();

        private native void allocate(@Cast({"cv::cuda::HostMem::AllocType"}) int i);

        private native void allocate(int i, int i2, int i3);

        private native void allocate(int i, int i2, int i3, @Cast({"cv::cuda::HostMem::AllocType"}) int i4);

        private native void allocate(@ByVal GpuMat gpuMat);

        private native void allocate(@ByVal GpuMat gpuMat, @Cast({"cv::cuda::HostMem::AllocType"}) int i);

        private native void allocate(@ByRef @Const HostMem hostMem);

        private native void allocate(@ByVal Mat mat);

        private native void allocate(@ByVal Mat mat, @Cast({"cv::cuda::HostMem::AllocType"}) int i);

        private native void allocate(@ByVal Size size, int i);

        private native void allocate(@ByVal Size size, int i, @Cast({"cv::cuda::HostMem::AllocType"}) int i2);

        private native void allocate(@ByVal UMat uMat);

        private native void allocate(@ByVal UMat uMat, @Cast({"cv::cuda::HostMem::AllocType"}) int i);

        private native void allocateArray(long j);

        public static native MatAllocator getAllocator();

        public static native MatAllocator getAllocator(@Cast({"cv::cuda::HostMem::AllocType"}) int i);

        @Cast({"cv::cuda::HostMem::AllocType"})
        public native int alloc_type();

        public native HostMem alloc_type(int i);

        public native int channels();

        @ByVal
        public native HostMem clone();

        public native int cols();

        public native HostMem cols(int i);

        public native void create(int i, int i2, int i3);

        public native void create(@ByVal Size size, int i);

        @ByVal
        public native GpuMat createGpuMatHeader();

        @ByVal
        public native Mat createMatHeader();

        @Cast({"uchar*"})
        public native BytePointer data();

        public native HostMem data(BytePointer bytePointer);

        @MemberGetter
        @Cast({"const uchar*"})
        public native BytePointer dataend();

        @Cast({"uchar*"})
        public native BytePointer datastart();

        public native HostMem datastart(BytePointer bytePointer);

        public native int depth();

        @Cast({"size_t"})
        public native long elemSize();

        @Cast({"size_t"})
        public native long elemSize1();

        @Cast({"bool"})
        public native boolean empty();

        public native int flags();

        public native HostMem flags(int i);

        @Cast({"bool"})
        public native boolean isContinuous();

        @ByRef
        @Name({"operator ="})
        public native HostMem put(@ByRef @Const HostMem hostMem);

        public native IntPointer refcount();

        public native HostMem refcount(IntPointer intPointer);

        public native void release();

        @ByVal
        public native HostMem reshape(int i);

        @ByVal
        public native HostMem reshape(int i, int i2);

        public native int rows();

        public native HostMem rows(int i);

        @ByVal
        public native Size size();

        @Cast({"size_t"})
        public native long step();

        public native HostMem step(long j);

        @Cast({"size_t"})
        public native long step1();

        public native void swap(@ByRef HostMem hostMem);

        public native int type();

        static {
            Loader.load();
        }

        public HostMem(Pointer p) {
            super(p);
        }

        public HostMem(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public HostMem position(long position) {
            return (HostMem) super.position(position);
        }

        public HostMem(@Cast({"cv::cuda::HostMem::AllocType"}) int alloc_type) {
            super((Pointer) null);
            allocate(alloc_type);
        }

        public HostMem() {
            super((Pointer) null);
            allocate();
        }

        public HostMem(@ByRef @Const HostMem m) {
            super((Pointer) null);
            allocate(m);
        }

        public HostMem(int rows, int cols, int type, @Cast({"cv::cuda::HostMem::AllocType"}) int alloc_type) {
            super((Pointer) null);
            allocate(rows, cols, type, alloc_type);
        }

        public HostMem(int rows, int cols, int type) {
            super((Pointer) null);
            allocate(rows, cols, type);
        }

        public HostMem(@ByVal Size size, int type, @Cast({"cv::cuda::HostMem::AllocType"}) int alloc_type) {
            super((Pointer) null);
            allocate(size, type, alloc_type);
        }

        public HostMem(@ByVal Size size, int type) {
            super((Pointer) null);
            allocate(size, type);
        }

        public HostMem(@ByVal Mat arr, @Cast({"cv::cuda::HostMem::AllocType"}) int alloc_type) {
            super((Pointer) null);
            allocate(arr, alloc_type);
        }

        public HostMem(@ByVal Mat arr) {
            super((Pointer) null);
            allocate(arr);
        }

        public HostMem(@ByVal UMat arr, @Cast({"cv::cuda::HostMem::AllocType"}) int alloc_type) {
            super((Pointer) null);
            allocate(arr, alloc_type);
        }

        public HostMem(@ByVal UMat arr) {
            super((Pointer) null);
            allocate(arr);
        }

        public HostMem(@ByVal GpuMat arr, @Cast({"cv::cuda::HostMem::AllocType"}) int alloc_type) {
            super((Pointer) null);
            allocate(arr, alloc_type);
        }

        public HostMem(@ByVal GpuMat arr) {
            super((Pointer) null);
            allocate(arr);
        }
    }

    @Namespace("cv::cuda")
    @NoOffset
    public static class Stream extends Pointer {
        @ByRef
        public static native Stream Null();

        private native void allocate();

        private native void allocate(@opencv_core.Ptr GpuMat.Allocator allocator);

        private native void allocateArray(long j);

        public native void enqueueHostCallback(StreamCallback streamCallback, Pointer pointer);

        @Cast({"bool"})
        public native boolean queryIfComplete();

        public native void waitEvent(@ByRef @Const Event event);

        public native void waitForCompletion();

        static {
            Loader.load();
        }

        public Stream(Pointer p) {
            super(p);
        }

        public Stream(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Stream position(long position) {
            return (Stream) super.position(position);
        }

        public static class StreamCallback extends FunctionPointer {
            private native void allocate();

            public native void call(int i, Pointer pointer);

            static {
                Loader.load();
            }

            public StreamCallback(Pointer p) {
                super(p);
            }

            protected StreamCallback() {
                allocate();
            }
        }

        public Stream() {
            super((Pointer) null);
            allocate();
        }

        public Stream(@opencv_core.Ptr GpuMat.Allocator allocator) {
            super((Pointer) null);
            allocate(allocator);
        }

        @Opaque
        public static class Impl extends Pointer {
            public Impl() {
                super((Pointer) null);
            }

            public Impl(Pointer p) {
                super(p);
            }
        }
    }

    @Namespace("cv::cuda")
    @NoOffset
    public static class Event extends Pointer {
        public static final int BLOCKING_SYNC = 1;
        public static final int DEFAULT = 0;
        public static final int DISABLE_TIMING = 2;
        public static final int INTERPROCESS = 4;

        private native void allocate();

        private native void allocate(@Cast({"cv::cuda::Event::CreateFlags"}) int i);

        private native void allocateArray(long j);

        public static native float elapsedTime(@ByRef @Const Event event, @ByRef @Const Event event2);

        @Cast({"bool"})
        public native boolean queryIfComplete();

        public native void record();

        public native void record(@ByRef(nullValue = "cv::cuda::Stream::Null()") Stream stream);

        public native void waitForCompletion();

        static {
            Loader.load();
        }

        public Event(Pointer p) {
            super(p);
        }

        public Event(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Event position(long position) {
            return (Event) super.position(position);
        }

        public Event(@Cast({"cv::cuda::Event::CreateFlags"}) int flags) {
            super((Pointer) null);
            allocate(flags);
        }

        public Event() {
            super((Pointer) null);
            allocate();
        }

        @Opaque
        public static class Impl extends Pointer {
            public Impl() {
                super((Pointer) null);
            }

            public Impl(Pointer p) {
                super(p);
            }
        }
    }

    @Namespace("cv::cuda")
    public static class TargetArchs extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"bool"})
        public static native boolean builtWith(@Cast({"cv::cuda::FeatureSet"}) int i);

        @Cast({"bool"})
        public static native boolean has(int i, int i2);

        @Cast({"bool"})
        public static native boolean hasBin(int i, int i2);

        @Cast({"bool"})
        public static native boolean hasEqualOrGreater(int i, int i2);

        @Cast({"bool"})
        public static native boolean hasEqualOrGreaterBin(int i, int i2);

        @Cast({"bool"})
        public static native boolean hasEqualOrGreaterPtx(int i, int i2);

        @Cast({"bool"})
        public static native boolean hasEqualOrLessPtx(int i, int i2);

        @Cast({"bool"})
        public static native boolean hasPtx(int i, int i2);

        static {
            Loader.load();
        }

        public TargetArchs() {
            super((Pointer) null);
            allocate();
        }

        public TargetArchs(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TargetArchs(Pointer p) {
            super(p);
        }

        public TargetArchs position(long position) {
            return (TargetArchs) super.position(position);
        }
    }

    @Namespace("cv::cuda")
    @NoOffset
    public static class DeviceInfo extends Pointer {
        public static final int ComputeModeDefault = 0;
        public static final int ComputeModeExclusive = 1;
        public static final int ComputeModeExclusiveProcess = 3;
        public static final int ComputeModeProhibited = 2;

        private native void allocate();

        private native void allocate(int i);

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean ECCEnabled();

        public native int asyncEngineCount();

        @Cast({"bool"})
        public native boolean canMapHostMemory();

        public native int clockRate();

        @Cast({"cv::cuda::DeviceInfo::ComputeMode"})
        public native int computeMode();

        @Cast({"bool"})
        public native boolean concurrentKernels();

        public native int deviceID();

        @Cast({"size_t"})
        public native long freeMemory();

        @Cast({"bool"})
        public native boolean integrated();

        @Cast({"bool"})
        public native boolean isCompatible();

        @Cast({"bool"})
        public native boolean kernelExecTimeoutEnabled();

        public native int l2CacheSize();

        public native int majorVersion();

        @ByVal
        public native Point3i maxGridSize();

        public native int maxSurface1D();

        @ByVal
        public native Point maxSurface1DLayered();

        @ByVal
        public native Point maxSurface2D();

        @ByVal
        public native Point3i maxSurface2DLayered();

        @ByVal
        public native Point3i maxSurface3D();

        public native int maxSurfaceCubemap();

        @ByVal
        public native Point maxSurfaceCubemapLayered();

        public native int maxTexture1D();

        @ByVal
        public native Point maxTexture1DLayered();

        public native int maxTexture1DLinear();

        public native int maxTexture1DMipmap();

        @ByVal
        public native Point maxTexture2D();

        @ByVal
        public native Point maxTexture2DGather();

        @ByVal
        public native Point3i maxTexture2DLayered();

        @ByVal
        public native Point3i maxTexture2DLinear();

        @ByVal
        public native Point maxTexture2DMipmap();

        @ByVal
        public native Point3i maxTexture3D();

        public native int maxTextureCubemap();

        @ByVal
        public native Point maxTextureCubemapLayered();

        @ByVal
        public native Point3i maxThreadsDim();

        public native int maxThreadsPerBlock();

        public native int maxThreadsPerMultiProcessor();

        @Cast({"size_t"})
        public native long memPitch();

        public native int memoryBusWidth();

        public native int memoryClockRate();

        public native int minorVersion();

        public native int multiProcessorCount();

        @Cast({"const char*"})
        public native BytePointer name();

        public native int pciBusID();

        public native int pciDeviceID();

        public native int pciDomainID();

        public native void queryMemory(@ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer, @ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer2);

        public native int regsPerBlock();

        @Cast({"size_t"})
        public native long sharedMemPerBlock();

        @Cast({"bool"})
        public native boolean supports(@Cast({"cv::cuda::FeatureSet"}) int i);

        @Cast({"size_t"})
        public native long surfaceAlignment();

        @Cast({"bool"})
        public native boolean tccDriver();

        @Cast({"size_t"})
        public native long textureAlignment();

        @Cast({"size_t"})
        public native long texturePitchAlignment();

        @Cast({"size_t"})
        public native long totalConstMem();

        @Cast({"size_t"})
        public native long totalGlobalMem();

        @Cast({"size_t"})
        public native long totalMemory();

        @Cast({"bool"})
        public native boolean unifiedAddressing();

        public native int warpSize();

        static {
            Loader.load();
        }

        public DeviceInfo(Pointer p) {
            super(p);
        }

        public DeviceInfo(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public DeviceInfo position(long position) {
            return (DeviceInfo) super.position(position);
        }

        public DeviceInfo() {
            super((Pointer) null);
            allocate();
        }

        public DeviceInfo(int device_id) {
            super((Pointer) null);
            allocate(device_id);
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class Device extends Pointer {
        public static final int EXEC_KERNEL = 1;
        public static final int EXEC_NATIVE_KERNEL = 2;
        public static final int FP_CORRECTLY_ROUNDED_DIVIDE_SQRT = 128;
        public static final int FP_DENORM = 1;
        public static final int FP_FMA = 32;
        public static final int FP_INF_NAN = 2;
        public static final int FP_ROUND_TO_INF = 16;
        public static final int FP_ROUND_TO_NEAREST = 4;
        public static final int FP_ROUND_TO_ZERO = 8;
        public static final int FP_SOFT_FLOAT = 64;
        public static final int LOCAL_IS_GLOBAL = 2;
        public static final int LOCAL_IS_LOCAL = 1;
        public static final int NO_CACHE = 0;
        public static final int NO_LOCAL_MEM = 0;
        public static final int READ_ONLY_CACHE = 1;
        public static final int READ_WRITE_CACHE = 2;
        public static final int TYPE_ACCELERATOR = 8;
        public static final int TYPE_ALL = -1;
        public static final int TYPE_CPU = 2;
        public static final int TYPE_DEFAULT = 1;
        public static final int TYPE_DGPU = 65540;
        public static final int TYPE_GPU = 4;
        public static final int TYPE_IGPU = 131076;
        public static final int UNKNOWN_VENDOR = 0;
        public static final int VENDOR_AMD = 1;
        public static final int VENDOR_INTEL = 2;
        public static final int VENDOR_NVIDIA = 3;

        private native void allocate();

        private native void allocate(Pointer pointer);

        private native void allocate(@ByRef @Const Device device);

        private native void allocateArray(long j);

        @ByRef
        @Const
        public static native Device getDefault();

        @opencv_core.Str
        public native BytePointer OpenCLVersion();

        @opencv_core.Str
        public native BytePointer OpenCL_C_Version();

        public native int addressBits();

        @Cast({"bool"})
        public native boolean available();

        @Cast({"bool"})
        public native boolean compilerAvailable();

        public native int deviceVersionMajor();

        public native int deviceVersionMinor();

        public native int doubleFPConfig();

        @opencv_core.Str
        public native BytePointer driverVersion();

        @Cast({"bool"})
        public native boolean endianLittle();

        @Cast({"bool"})
        public native boolean errorCorrectionSupport();

        public native int executionCapabilities();

        @opencv_core.Str
        public native BytePointer extensions();

        public native int globalMemCacheLineSize();

        @Cast({"size_t"})
        public native long globalMemCacheSize();

        public native int globalMemCacheType();

        @Cast({"size_t"})
        public native long globalMemSize();

        public native int halfFPConfig();

        @Cast({"bool"})
        public native boolean hostUnifiedMemory();

        @Cast({"size_t"})
        public native long image2DMaxHeight();

        @Cast({"size_t"})
        public native long image2DMaxWidth();

        @Cast({"size_t"})
        public native long image3DMaxDepth();

        @Cast({"size_t"})
        public native long image3DMaxHeight();

        @Cast({"size_t"})
        public native long image3DMaxWidth();

        @Cast({"uint"})
        public native int imageBaseAddressAlignment();

        @Cast({"bool"})
        public native boolean imageFromBufferSupport();

        @Cast({"size_t"})
        public native long imageMaxArraySize();

        @Cast({"size_t"})
        public native long imageMaxBufferSize();

        @Cast({"uint"})
        public native int imagePitchAlignment();

        @Cast({"bool"})
        public native boolean imageSupport();

        @Cast({"bool"})
        public native boolean intelSubgroupsSupport();

        @Cast({"bool"})
        public native boolean isAMD();

        @Cast({"bool"})
        public native boolean isExtensionSupported(@opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean isExtensionSupported(@opencv_core.Str BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean isIntel();

        @Cast({"bool"})
        public native boolean isNVidia();

        @Cast({"bool"})
        public native boolean linkerAvailable();

        @Cast({"size_t"})
        public native long localMemSize();

        public native int localMemType();

        public native int maxClockFrequency();

        public native int maxComputeUnits();

        public native int maxConstantArgs();

        @Cast({"size_t"})
        public native long maxConstantBufferSize();

        @Cast({"size_t"})
        public native long maxMemAllocSize();

        @Cast({"size_t"})
        public native long maxParameterSize();

        public native int maxReadImageArgs();

        public native int maxSamplers();

        @Cast({"size_t"})
        public native long maxWorkGroupSize();

        public native int maxWorkItemDims();

        public native void maxWorkItemSizes(@Cast({"size_t*"}) SizeTPointer sizeTPointer);

        public native int maxWriteImageArgs();

        public native int memBaseAddrAlign();

        @opencv_core.Str
        public native BytePointer name();

        public native int nativeVectorWidthChar();

        public native int nativeVectorWidthDouble();

        public native int nativeVectorWidthFloat();

        public native int nativeVectorWidthHalf();

        public native int nativeVectorWidthInt();

        public native int nativeVectorWidthLong();

        public native int nativeVectorWidthShort();

        public native int preferredVectorWidthChar();

        public native int preferredVectorWidthDouble();

        public native int preferredVectorWidthFloat();

        public native int preferredVectorWidthHalf();

        public native int preferredVectorWidthInt();

        public native int preferredVectorWidthLong();

        public native int preferredVectorWidthShort();

        @Cast({"size_t"})
        public native long printfBufferSize();

        @Cast({"size_t"})
        public native long profilingTimerResolution();

        public native Pointer ptr();

        @ByRef
        @Name({"operator ="})
        public native Device put(@ByRef @Const Device device);

        public native void set(Pointer pointer);

        public native int singleFPConfig();

        public native int type();

        public native int vendorID();

        @opencv_core.Str
        public native BytePointer vendorName();

        @opencv_core.Str
        public native BytePointer version();

        static {
            Loader.load();
        }

        public Device(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Device position(long position) {
            return (Device) super.position(position);
        }

        public Device() {
            super((Pointer) null);
            allocate();
        }

        public Device(Pointer d) {
            super((Pointer) null);
            allocate(d);
        }

        public Device(@ByRef @Const Device d) {
            super((Pointer) null);
            allocate(d);
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class Context extends Pointer {
        private native void allocate();

        private native void allocate(int i);

        private native void allocate(@ByRef @Const Context context);

        private native void allocateArray(long j);

        @ByRef
        public static native Context getDefault();

        @ByRef
        public static native Context getDefault(@Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean create();

        @Cast({"bool"})
        public native boolean create(int i);

        @ByRef
        @Const
        public native Device device(@Cast({"size_t"}) long j);

        public native Impl getImpl();

        @ByVal
        public native Program getProg(@ByRef @Const ProgramSource programSource, @opencv_core.Str String str, @opencv_core.Str String str2);

        @ByVal
        public native Program getProg(@ByRef @Const ProgramSource programSource, @opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @Cast({"size_t"})
        public native long ndevices();

        public native Impl p();

        public native Context p(Impl impl);

        public native Pointer ptr();

        @ByRef
        @Name({"operator ="})
        public native Context put(@ByRef @Const Context context);

        public native void setUseSVM(@Cast({"bool"}) boolean z);

        public native void unloadProg(@ByRef Program program);

        @Cast({"bool"})
        public native boolean useSVM();

        static {
            Loader.load();
        }

        public Context(Pointer p) {
            super(p);
        }

        public Context(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Context position(long position) {
            return (Context) super.position(position);
        }

        public Context() {
            super((Pointer) null);
            allocate();
        }

        public Context(int dtype) {
            super((Pointer) null);
            allocate(dtype);
        }

        public Context(@ByRef @Const Context c) {
            super((Pointer) null);
            allocate(c);
        }

        @Opaque
        public static class Impl extends Pointer {
            public Impl() {
                super((Pointer) null);
            }

            public Impl(Pointer p) {
                super(p);
            }
        }
    }

    @Name({"cv::ocl::Platform"})
    @NoOffset
    public static class OclPlatform extends Pointer {
        private native void allocate();

        private native void allocate(@ByRef @Const OclPlatform oclPlatform);

        private native void allocateArray(long j);

        @ByRef
        public static native OclPlatform getDefault();

        public native Pointer ptr();

        @ByRef
        @Name({"operator ="})
        public native OclPlatform put(@ByRef @Const OclPlatform oclPlatform);

        static {
            Loader.load();
        }

        public OclPlatform(Pointer p) {
            super(p);
        }

        public OclPlatform(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public OclPlatform position(long position) {
            return (OclPlatform) super.position(position);
        }

        public OclPlatform() {
            super((Pointer) null);
            allocate();
        }

        public OclPlatform(@ByRef @Const OclPlatform p) {
            super((Pointer) null);
            allocate(p);
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class Queue extends Pointer {
        private native void allocate();

        private native void allocate(@ByRef @Const Context context);

        private native void allocate(@ByRef @Const Context context, @ByRef(nullValue = "cv::ocl::Device()") @Const Device device);

        private native void allocate(@ByRef @Const Queue queue);

        private native void allocateArray(long j);

        @ByRef
        public static native Queue getDefault();

        @Cast({"bool"})
        public native boolean create();

        @Cast({"bool"})
        public native boolean create(@ByRef(nullValue = "cv::ocl::Context()") @Const Context context, @ByRef(nullValue = "cv::ocl::Device()") @Const Device device);

        public native void finish();

        @Cast({"cv::ocl::Queue::Impl*"})
        public native Pointer getImpl();

        @ByRef
        @Const
        public native Queue getProfilingQueue();

        public native Pointer ptr();

        @ByRef
        @Name({"operator ="})
        public native Queue put(@ByRef @Const Queue queue);

        static {
            Loader.load();
        }

        public Queue(Pointer p) {
            super(p);
        }

        public Queue(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Queue position(long position) {
            return (Queue) super.position(position);
        }

        public Queue() {
            super((Pointer) null);
            allocate();
        }

        public Queue(@ByRef @Const Context c, @ByRef(nullValue = "cv::ocl::Device()") @Const Device d) {
            super((Pointer) null);
            allocate(c, d);
        }

        public Queue(@ByRef @Const Context c) {
            super((Pointer) null);
            allocate(c);
        }

        public Queue(@ByRef @Const Queue q) {
            super((Pointer) null);
            allocate(q);
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class KernelArg extends Pointer {
        public static final int CONSTANT = 8;
        public static final int LOCAL = 1;
        public static final int NO_SIZE = 256;
        public static final int PTR_ONLY = 16;
        public static final int READ_ONLY = 2;
        public static final int READ_WRITE = 6;
        public static final int WRITE_ONLY = 4;

        @ByVal
        public static native KernelArg Constant(@ByRef @Const Mat mat);

        @ByVal
        public static native KernelArg Local(@Cast({"size_t"}) long j);

        @ByVal
        public static native KernelArg PtrReadOnly(@ByRef @Const UMat uMat);

        @ByVal
        public static native KernelArg PtrReadWrite(@ByRef @Const UMat uMat);

        @ByVal
        public static native KernelArg PtrWriteOnly(@ByRef @Const UMat uMat);

        @ByVal
        public static native KernelArg ReadOnly(@ByRef @Const UMat uMat);

        @ByVal
        public static native KernelArg ReadOnly(@ByRef @Const UMat uMat, int i, int i2);

        @ByVal
        public static native KernelArg ReadOnlyNoSize(@ByRef @Const UMat uMat);

        @ByVal
        public static native KernelArg ReadOnlyNoSize(@ByRef @Const UMat uMat, int i, int i2);

        @ByVal
        public static native KernelArg ReadWrite(@ByRef @Const UMat uMat);

        @ByVal
        public static native KernelArg ReadWrite(@ByRef @Const UMat uMat, int i, int i2);

        @ByVal
        public static native KernelArg ReadWriteNoSize(@ByRef @Const UMat uMat);

        @ByVal
        public static native KernelArg ReadWriteNoSize(@ByRef @Const UMat uMat, int i, int i2);

        @ByVal
        public static native KernelArg WriteOnly(@ByRef @Const UMat uMat);

        @ByVal
        public static native KernelArg WriteOnly(@ByRef @Const UMat uMat, int i, int i2);

        @ByVal
        public static native KernelArg WriteOnlyNoSize(@ByRef @Const UMat uMat);

        @ByVal
        public static native KernelArg WriteOnlyNoSize(@ByRef @Const UMat uMat, int i, int i2);

        private native void allocate();

        private native void allocate(int i, UMat uMat);

        private native void allocate(int i, UMat uMat, int i2, int i3, @Const Pointer pointer, @Cast({"size_t"}) long j);

        private native void allocateArray(long j);

        public native int flags();

        public native KernelArg flags(int i);

        public native int iwscale();

        public native KernelArg iwscale(int i);

        public native KernelArg m(UMat uMat);

        public native UMat m();

        @MemberGetter
        @Const
        public native Pointer obj();

        @Cast({"size_t"})
        public native long sz();

        public native KernelArg sz(long j);

        public native int wscale();

        public native KernelArg wscale(int i);

        static {
            Loader.load();
        }

        public KernelArg(Pointer p) {
            super(p);
        }

        public KernelArg(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public KernelArg position(long position) {
            return (KernelArg) super.position(position);
        }

        public KernelArg(int _flags, UMat _m, int wscale, int iwscale, @Const Pointer _obj, @Cast({"size_t"}) long _sz) {
            super((Pointer) null);
            allocate(_flags, _m, wscale, iwscale, _obj, _sz);
        }

        public KernelArg(int _flags, UMat _m) {
            super((Pointer) null);
            allocate(_flags, _m);
        }

        public KernelArg() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class Kernel extends Pointer {
        private native void allocate();

        private native void allocate(String str, @ByRef @Const Program program);

        private native void allocate(String str, @ByRef @Const ProgramSource programSource, @opencv_core.Str String str2, @opencv_core.Str @Cast({"", "cv::String*"}) BytePointer bytePointer);

        private native void allocate(@Cast({"const char*"}) BytePointer bytePointer, @ByRef @Const Program program);

        private native void allocate(@ByRef @Const Kernel kernel);

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean compileWorkGroupSize(@Cast({"size_t*"}) SizeTPointer sizeTPointer);

        @Cast({"bool"})
        public native boolean create(String str, @ByRef @Const Program program);

        @Cast({"bool"})
        public native boolean create(String str, @ByRef @Const ProgramSource programSource, @opencv_core.Str String str2, @opencv_core.Str @Cast({"", "cv::String*"}) BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean create(@Cast({"const char*"}) BytePointer bytePointer, @ByRef @Const Program program);

        @Cast({"bool"})
        public native boolean empty();

        @Cast({"size_t"})
        public native long localMemSize();

        @Cast({"size_t"})
        public native long preferedWorkGroupSizeMultiple();

        public native Pointer ptr();

        @ByRef
        @Name({"operator ="})
        public native Kernel put(@ByRef @Const Kernel kernel);

        @Cast({"bool"})
        public native boolean run(int i, @Cast({"size_t*"}) SizeTPointer sizeTPointer, @Cast({"size_t*"}) SizeTPointer sizeTPointer2, @Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean run(int i, @Cast({"size_t*"}) SizeTPointer sizeTPointer, @Cast({"size_t*"}) SizeTPointer sizeTPointer2, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::ocl::Queue()") @Const Queue queue);

        @Cast({"int64"})
        public native long runProfiling(int i, @Cast({"size_t*"}) SizeTPointer sizeTPointer, @Cast({"size_t*"}) SizeTPointer sizeTPointer2);

        @Cast({"int64"})
        public native long runProfiling(int i, @Cast({"size_t*"}) SizeTPointer sizeTPointer, @Cast({"size_t*"}) SizeTPointer sizeTPointer2, @ByRef(nullValue = "cv::ocl::Queue()") @Const Queue queue);

        @Cast({"bool"})
        public native boolean runTask(@Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean runTask(@Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::ocl::Queue()") @Const Queue queue);

        public native int set(int i, @Const Pointer pointer, @Cast({"size_t"}) long j);

        public native int set(int i, @ByRef @Const Image2D image2D);

        public native int set(int i, @ByRef @Const KernelArg kernelArg);

        public native int set(int i, @ByRef @Const UMat uMat);

        @Cast({"size_t"})
        public native long workGroupSize();

        static {
            Loader.load();
        }

        public Kernel(Pointer p) {
            super(p);
        }

        public Kernel(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Kernel position(long position) {
            return (Kernel) super.position(position);
        }

        public Kernel() {
            super((Pointer) null);
            allocate();
        }

        public Kernel(@Cast({"const char*"}) BytePointer kname, @ByRef @Const Program prog) {
            super((Pointer) null);
            allocate(kname, prog);
        }

        public Kernel(String kname, @ByRef @Const Program prog) {
            super((Pointer) null);
            allocate(kname, prog);
        }

        public Kernel(String kname, @ByRef @Const ProgramSource prog, @opencv_core.Str String buildopts, @opencv_core.Str BytePointer errmsg) {
            allocate(kname, prog, buildopts, errmsg);
        }

        public Kernel(@ByRef @Const Kernel k) {
            super((Pointer) null);
            allocate(k);
        }

        @Opaque
        public static class Impl extends Pointer {
            public Impl() {
                super((Pointer) null);
            }

            public Impl(Pointer p) {
                super(p);
            }
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class Program extends Pointer {
        private native void allocate();

        private native void allocate(@ByRef @Const Program program);

        private native void allocate(@ByRef @Const ProgramSource programSource, @opencv_core.Str String str, @opencv_core.Str String str2);

        private native void allocate(@ByRef @Const ProgramSource programSource, @opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        private native void allocateArray(long j);

        @opencv_core.Str
        @Deprecated
        public static native String getPrefix(@opencv_core.Str String str);

        @opencv_core.Str
        @Deprecated
        public static native BytePointer getPrefix(@opencv_core.Str BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean create(@ByRef @Const ProgramSource programSource, @opencv_core.Str String str, @opencv_core.Str String str2);

        @Cast({"bool"})
        public native boolean create(@ByRef @Const ProgramSource programSource, @opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        public native void getBinary(@Cast({"char*"}) @StdVector ByteBuffer byteBuffer);

        public native void getBinary(@Cast({"char*"}) @StdVector BytePointer bytePointer);

        public native void getBinary(@Cast({"char*"}) @StdVector byte[] bArr);

        @Cast({"cv::ocl::Program::Impl*"})
        public native Pointer getImpl();

        @opencv_core.Str
        @Deprecated
        public native BytePointer getPrefix();

        public native Pointer ptr();

        @ByRef
        @Name({"operator ="})
        public native Program put(@ByRef @Const Program program);

        @Deprecated
        @Cast({"bool"})
        public native boolean read(@opencv_core.Str String str, @opencv_core.Str String str2);

        @Deprecated
        @Cast({"bool"})
        public native boolean read(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @ByRef
        @Deprecated
        @Const
        public native ProgramSource source();

        @Deprecated
        @Cast({"bool"})
        public native boolean write(@opencv_core.Str String str);

        @Deprecated
        @Cast({"bool"})
        public native boolean write(@opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public Program(Pointer p) {
            super(p);
        }

        public Program(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Program position(long position) {
            return (Program) super.position(position);
        }

        public Program() {
            super((Pointer) null);
            allocate();
        }

        public Program(@ByRef @Const ProgramSource src, @opencv_core.Str BytePointer buildflags, @opencv_core.Str BytePointer errmsg) {
            super((Pointer) null);
            allocate(src, buildflags, errmsg);
        }

        public Program(@ByRef @Const ProgramSource src, @opencv_core.Str String buildflags, @opencv_core.Str String errmsg) {
            super((Pointer) null);
            allocate(src, buildflags, errmsg);
        }

        public Program(@ByRef @Const Program prog) {
            super((Pointer) null);
            allocate(prog);
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class ProgramSource extends Pointer {
        private native void allocate();

        private native void allocate(@opencv_core.Str String str);

        private native void allocate(@opencv_core.Str String str, @opencv_core.Str String str2, @opencv_core.Str String str3, @opencv_core.Str String str4);

        private native void allocate(@opencv_core.Str BytePointer bytePointer);

        private native void allocate(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @opencv_core.Str BytePointer bytePointer3, @opencv_core.Str BytePointer bytePointer4);

        private native void allocate(@ByRef @Const ProgramSource programSource);

        private native void allocateArray(long j);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) ByteBuffer byteBuffer, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) ByteBuffer byteBuffer, @Cast({"const size_t"}) long j, @opencv_core.Str String str3);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) BytePointer bytePointer, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) BytePointer bytePointer, @Cast({"const size_t"}) long j, @opencv_core.Str String str3);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) byte[] bArr, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) byte[] bArr, @Cast({"const size_t"}) long j, @opencv_core.Str String str3);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) ByteBuffer byteBuffer, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) ByteBuffer byteBuffer, @Cast({"const size_t"}) long j, @opencv_core.Str BytePointer bytePointer3);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) BytePointer bytePointer3, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) BytePointer bytePointer3, @Cast({"const size_t"}) long j, @opencv_core.Str BytePointer bytePointer4);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) byte[] bArr, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromBinary(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) byte[] bArr, @Cast({"const size_t"}) long j, @opencv_core.Str BytePointer bytePointer3);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) ByteBuffer byteBuffer, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) ByteBuffer byteBuffer, @Cast({"const size_t"}) long j, @opencv_core.Str String str3);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) BytePointer bytePointer, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) BytePointer bytePointer, @Cast({"const size_t"}) long j, @opencv_core.Str String str3);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) byte[] bArr, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str String str, @opencv_core.Str String str2, @Cast({"const unsigned char*"}) byte[] bArr, @Cast({"const size_t"}) long j, @opencv_core.Str String str3);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) ByteBuffer byteBuffer, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) ByteBuffer byteBuffer, @Cast({"const size_t"}) long j, @opencv_core.Str BytePointer bytePointer3);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) BytePointer bytePointer3, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) BytePointer bytePointer3, @Cast({"const size_t"}) long j, @opencv_core.Str BytePointer bytePointer4);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) byte[] bArr, @Cast({"const size_t"}) long j);

        @ByVal
        public static native ProgramSource fromSPIR(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @Cast({"const unsigned char*"}) byte[] bArr, @Cast({"const size_t"}) long j, @opencv_core.Str BytePointer bytePointer3);

        @Cast({"cv::ocl::ProgramSource::Impl*"})
        public native Pointer getImpl();

        @Cast({"cv::ocl::ProgramSource::hash_t"})
        public native int hash();

        @ByRef
        @Name({"operator ="})
        public native ProgramSource put(@ByRef @Const ProgramSource programSource);

        @opencv_core.Str
        public native BytePointer source();

        static {
            Loader.load();
        }

        public ProgramSource(Pointer p) {
            super(p);
        }

        public ProgramSource(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public ProgramSource position(long position) {
            return (ProgramSource) super.position(position);
        }

        public ProgramSource() {
            super((Pointer) null);
            allocate();
        }

        public ProgramSource(@opencv_core.Str BytePointer module, @opencv_core.Str BytePointer name, @opencv_core.Str BytePointer codeStr, @opencv_core.Str BytePointer codeHash) {
            super((Pointer) null);
            allocate(module, name, codeStr, codeHash);
        }

        public ProgramSource(@opencv_core.Str String module, @opencv_core.Str String name, @opencv_core.Str String codeStr, @opencv_core.Str String codeHash) {
            super((Pointer) null);
            allocate(module, name, codeStr, codeHash);
        }

        public ProgramSource(@opencv_core.Str BytePointer prog) {
            super((Pointer) null);
            allocate(prog);
        }

        public ProgramSource(@opencv_core.Str String prog) {
            super((Pointer) null);
            allocate(prog);
        }

        public ProgramSource(@ByRef @Const ProgramSource prog) {
            super((Pointer) null);
            allocate(prog);
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class PlatformInfo extends Pointer {
        private native void allocate();

        private native void allocate(Pointer pointer);

        private native void allocate(@ByRef @Const PlatformInfo platformInfo);

        private native void allocateArray(long j);

        public native int deviceNumber();

        public native void getDevice(@ByRef Device device, int i);

        @opencv_core.Str
        public native BytePointer name();

        @ByRef
        @Name({"operator ="})
        public native PlatformInfo put(@ByRef @Const PlatformInfo platformInfo);

        @opencv_core.Str
        public native BytePointer vendor();

        @opencv_core.Str
        public native BytePointer version();

        static {
            Loader.load();
        }

        public PlatformInfo(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public PlatformInfo position(long position) {
            return (PlatformInfo) super.position(position);
        }

        public PlatformInfo() {
            super((Pointer) null);
            allocate();
        }

        public PlatformInfo(Pointer id) {
            super((Pointer) null);
            allocate(id);
        }

        public PlatformInfo(@ByRef @Const PlatformInfo i) {
            super((Pointer) null);
            allocate(i);
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class Image2D extends Pointer {
        private native void allocate();

        private native void allocate(@ByRef @Const Image2D image2D);

        private native void allocate(@ByRef @Const UMat uMat);

        private native void allocate(@ByRef @Const UMat uMat, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2);

        private native void allocateArray(long j);

        @Cast({"bool"})
        public static native boolean canCreateAlias(@ByRef @Const UMat uMat);

        @Cast({"bool"})
        public static native boolean isFormatSupported(int i, int i2, @Cast({"bool"}) boolean z);

        public native Pointer ptr();

        @ByRef
        @Name({"operator ="})
        public native Image2D put(@ByRef @Const Image2D image2D);

        static {
            Loader.load();
        }

        public Image2D(Pointer p) {
            super(p);
        }

        public Image2D(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Image2D position(long position) {
            return (Image2D) super.position(position);
        }

        public Image2D() {
            super((Pointer) null);
            allocate();
        }

        public Image2D(@ByRef @Const UMat src, @Cast({"bool"}) boolean norm, @Cast({"bool"}) boolean alias) {
            super((Pointer) null);
            allocate(src, norm, alias);
        }

        public Image2D(@ByRef @Const UMat src) {
            super((Pointer) null);
            allocate(src);
        }

        public Image2D(@ByRef @Const Image2D i) {
            super((Pointer) null);
            allocate(i);
        }
    }

    @Namespace("cv::ocl")
    @NoOffset
    public static class Timer extends Pointer {
        private native void allocate(@ByRef @Const Queue queue);

        @Cast({"uint64"})
        public native int durationNS();

        public native void start();

        public native void stop();

        static {
            Loader.load();
        }

        public Timer(Pointer p) {
            super(p);
        }

        public Timer(@ByRef @Const Queue q) {
            super((Pointer) null);
            allocate(q);
        }
    }

    @Namespace("cv")
    public static class BufferPoolController extends Pointer {
        public native void freeAllReservedBuffers();

        @Cast({"size_t"})
        public native long getMaxReservedSize();

        @Cast({"size_t"})
        public native long getReservedSize();

        public native void setMaxReservedSize(@Cast({"size_t"}) long j);

        static {
            Loader.load();
        }

        public BufferPoolController(Pointer p) {
            super(p);
        }
    }

    public static Mat noArray() {
        return null;
    }

    @Namespace("cv")
    public static class MatAllocator extends Pointer {
        @Name({"allocate"})
        public native UMatData _allocate(int i, @Const IntBuffer intBuffer, int i2, Pointer pointer, @Cast({"size_t*"}) SizeTPointer sizeTPointer, @Cast({"cv::AccessFlag"}) int i3, @Cast({"cv::UMatUsageFlags"}) int i4);

        @Name({"allocate"})
        public native UMatData _allocate(int i, @Const IntPointer intPointer, int i2, Pointer pointer, @Cast({"size_t*"}) SizeTPointer sizeTPointer, @Cast({"cv::AccessFlag"}) int i3, @Cast({"cv::UMatUsageFlags"}) int i4);

        @Name({"allocate"})
        public native UMatData _allocate(int i, @Const int[] iArr, int i2, Pointer pointer, @Cast({"size_t*"}) SizeTPointer sizeTPointer, @Cast({"cv::AccessFlag"}) int i3, @Cast({"cv::UMatUsageFlags"}) int i4);

        @Cast({"bool"})
        @Name({"allocate"})
        public native boolean _allocate(UMatData uMatData, @Cast({"cv::AccessFlag"}) int i, @Cast({"cv::UMatUsageFlags"}) int i2);

        @Name({"deallocate"})
        public native void _deallocate(UMatData uMatData);

        public native void copy(UMatData uMatData, UMatData uMatData2, int i, @Cast({"const size_t*"}) SizeTPointer sizeTPointer, @Cast({"const size_t*"}) SizeTPointer sizeTPointer2, @Cast({"const size_t*"}) SizeTPointer sizeTPointer3, @Cast({"const size_t*"}) SizeTPointer sizeTPointer4, @Cast({"const size_t*"}) SizeTPointer sizeTPointer5, @Cast({"bool"}) boolean z);

        public native void download(UMatData uMatData, Pointer pointer, int i, @Cast({"const size_t*"}) SizeTPointer sizeTPointer, @Cast({"const size_t*"}) SizeTPointer sizeTPointer2, @Cast({"const size_t*"}) SizeTPointer sizeTPointer3, @Cast({"const size_t*"}) SizeTPointer sizeTPointer4);

        public native BufferPoolController getBufferPoolController();

        public native BufferPoolController getBufferPoolController(String str);

        public native BufferPoolController getBufferPoolController(@Cast({"const char*"}) BytePointer bytePointer);

        public native void map(UMatData uMatData, @Cast({"cv::AccessFlag"}) int i);

        public native void unmap(UMatData uMatData);

        public native void upload(UMatData uMatData, @Const Pointer pointer, int i, @Cast({"const size_t*"}) SizeTPointer sizeTPointer, @Cast({"const size_t*"}) SizeTPointer sizeTPointer2, @Cast({"const size_t*"}) SizeTPointer sizeTPointer3, @Cast({"const size_t*"}) SizeTPointer sizeTPointer4);

        static {
            Loader.load();
        }

        public MatAllocator(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class UMatData extends Pointer {
        public static final int ASYNC_CLEANUP = 128;
        public static final int COPY_ON_MAP = 1;
        public static final int DEVICE_COPY_OBSOLETE = 4;
        public static final int DEVICE_MEM_MAPPED = 64;
        public static final int HOST_COPY_OBSOLETE = 2;
        public static final int TEMP_COPIED_UMAT = 24;
        public static final int TEMP_UMAT = 8;
        public static final int USER_ALLOCATED = 32;

        private native void allocate(@Const MatAllocator matAllocator);

        public native int allocatorFlags_();

        public native UMatData allocatorFlags_(int i);

        @Cast({"bool"})
        public native boolean copyOnMap();

        @MemberGetter
        @Const
        public native MatAllocator currAllocator();

        @Cast({"uchar*"})
        public native BytePointer data();

        public native UMatData data(BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean deviceCopyObsolete();

        @Cast({"bool"})
        public native boolean deviceMemMapped();

        @Cast({"cv::UMatData::MemoryFlag"})
        public native int flags();

        public native UMatData flags(int i);

        public native Pointer handle();

        public native UMatData handle(Pointer pointer);

        @Cast({"bool"})
        public native boolean hostCopyObsolete();

        public native void lock();

        public native int mapcount();

        public native UMatData mapcount(int i);

        public native void markDeviceCopyObsolete(@Cast({"bool"}) boolean z);

        public native void markDeviceMemMapped(@Cast({"bool"}) boolean z);

        public native void markHostCopyObsolete(@Cast({"bool"}) boolean z);

        @Cast({"uchar*"})
        public native BytePointer origdata();

        public native UMatData origdata(BytePointer bytePointer);

        public native UMatData originalUMatData();

        public native UMatData originalUMatData(UMatData uMatData);

        @MemberGetter
        @Const
        public native MatAllocator prevAllocator();

        public native int refcount();

        public native UMatData refcount(int i);

        @Cast({"size_t"})
        public native long size();

        public native UMatData size(long j);

        @Cast({"bool"})
        public native boolean tempCopiedUMat();

        @Cast({"bool"})
        public native boolean tempUMat();

        public native void unlock();

        public native int urefcount();

        public native UMatData urefcount(int i);

        public native Pointer userdata();

        public native UMatData userdata(Pointer pointer);

        static {
            Loader.load();
        }

        public UMatData(Pointer p) {
            super(p);
        }

        public UMatData(@Const MatAllocator allocator) {
            super((Pointer) null);
            allocate(allocator);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class MatSize extends Pointer {
        private native void allocate(IntBuffer intBuffer);

        private native void allocate(IntPointer intPointer);

        private native void allocate(int[] iArr);

        @ByVal
        @Name({"operator ()"})
        public native Size apply();

        @Const
        @Name({"operator const int*"})
        public native IntPointer asIntPointer();

        public native int dims();

        @Cast({"bool"})
        @Name({"operator =="})
        public native boolean equals(@ByRef @Const MatSize matSize);

        @ByRef
        @Name({"operator []"})
        public native IntPointer get(int i);

        @Cast({"bool"})
        @Name({"operator !="})
        public native boolean notEquals(@ByRef @Const MatSize matSize);

        public native IntPointer p();

        public native MatSize p(IntPointer intPointer);

        static {
            Loader.load();
        }

        public MatSize(Pointer p) {
            super(p);
        }

        public MatSize(IntPointer _p) {
            super((Pointer) null);
            allocate(_p);
        }

        public MatSize(IntBuffer _p) {
            super((Pointer) null);
            allocate(_p);
        }

        public MatSize(int[] _p) {
            super((Pointer) null);
            allocate(_p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class MatStep extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @Cast({"size_t"})
        @Name({"operator size_t"})
        public native long asLong();

        @Cast({"size_t"})
        public native long buf(int i);

        @MemberGetter
        @Cast({"size_t*"})
        public native SizeTPointer buf();

        public native MatStep buf(int i, long j);

        @ByRef
        @Cast({"size_t*"})
        @Name({"operator []"})
        public native SizeTPointer get(int i);

        @Cast({"size_t*"})
        public native SizeTPointer p();

        public native MatStep p(SizeTPointer sizeTPointer);

        @ByRef
        @Name({"operator ="})
        public native MatStep put(@Cast({"size_t"}) long j);

        static {
            Loader.load();
        }

        public MatStep(Pointer p) {
            super(p);
        }

        public MatStep() {
            super((Pointer) null);
            allocate();
        }

        public MatStep(@Cast({"size_t"}) long s) {
            super((Pointer) null);
            allocate(s);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class Mat extends opencv_core.AbstractMat {
        public static final int AUTO_STEP = 0;
        public static final int CONTINUOUS_FLAG = 16384;
        public static final int DEPTH_MASK = 7;
        public static final int MAGIC_MASK = -65536;
        public static final int MAGIC_VAL = 1124007936;
        public static final int SUBMATRIX_FLAG = 32768;
        public static final int TYPE_MASK = 4095;
        private Pointer pointer;

        private native void allocate();

        private native void allocate(int i, int i2, int i3);

        private native void allocate(int i, int i2, int i3, Pointer pointer2, @Cast({"size_t"}) long j);

        private native void allocate(int i, int i2, int i3, @ByRef @Const Scalar scalar);

        private native void allocate(int i, @Const IntBuffer intBuffer, int i2);

        private native void allocate(int i, @Const IntBuffer intBuffer, int i2, Pointer pointer2);

        private native void allocate(int i, @Const IntBuffer intBuffer, int i2, Pointer pointer2, @Cast({"const size_t*"}) SizeTPointer sizeTPointer);

        private native void allocate(int i, @Const IntBuffer intBuffer, int i2, @ByRef @Const Scalar scalar);

        private native void allocate(int i, @Const IntPointer intPointer, int i2);

        private native void allocate(int i, @Const IntPointer intPointer, int i2, Pointer pointer2);

        private native void allocate(int i, @Const IntPointer intPointer, int i2, Pointer pointer2, @Cast({"const size_t*"}) SizeTPointer sizeTPointer);

        private native void allocate(int i, @Const IntPointer intPointer, int i2, @ByRef @Const Scalar scalar);

        private native void allocate(int i, @Const int[] iArr, int i2);

        private native void allocate(int i, @Const int[] iArr, int i2, Pointer pointer2);

        private native void allocate(int i, @Const int[] iArr, int i2, Pointer pointer2, @Cast({"const size_t*"}) SizeTPointer sizeTPointer);

        private native void allocate(int i, @Const int[] iArr, int i2, @ByRef @Const Scalar scalar);

        private native void allocate(@StdVector IntBuffer intBuffer, int i);

        private native void allocate(@StdVector IntBuffer intBuffer, int i, Pointer pointer2);

        private native void allocate(@StdVector IntBuffer intBuffer, int i, Pointer pointer2, @Cast({"const size_t*"}) SizeTPointer sizeTPointer);

        private native void allocate(@StdVector IntBuffer intBuffer, int i, @ByRef @Const Scalar scalar);

        private native void allocate(@StdVector IntPointer intPointer, int i);

        private native void allocate(@StdVector IntPointer intPointer, int i, Pointer pointer2);

        private native void allocate(@StdVector IntPointer intPointer, int i, Pointer pointer2, @Cast({"const size_t*"}) SizeTPointer sizeTPointer);

        private native void allocate(@StdVector IntPointer intPointer, int i, @ByRef @Const Scalar scalar);

        private native void allocate(@ByRef @Const GpuMat gpuMat);

        private native void allocate(@ByRef @Const Mat mat);

        private native void allocate(@ByRef @Const Mat mat, @ByRef @Const Range range);

        private native void allocate(@ByRef @Const Mat mat, @ByRef @Const Range range, @ByRef(nullValue = "cv::Range::all()") @Const Range range2);

        private native void allocate(@ByRef @Const Mat mat, @ByRef @Const Rect rect);

        private native void allocate(@ByVal Size size, int i);

        private native void allocate(@ByVal Size size, int i, Pointer pointer2);

        private native void allocate(@ByVal Size size, int i, Pointer pointer2, @Cast({"size_t"}) long j);

        private native void allocate(@ByVal Size size, int i, @ByRef @Const Scalar scalar);

        private native void allocate(@StdVector int[] iArr, int i);

        private native void allocate(@StdVector int[] iArr, int i, Pointer pointer2);

        private native void allocate(@StdVector int[] iArr, int i, Pointer pointer2, @Cast({"const size_t*"}) SizeTPointer sizeTPointer);

        private native void allocate(@StdVector int[] iArr, int i, @ByRef @Const Scalar scalar);

        private native void allocateArray(long j);

        @ByVal
        public static native Mat diag(@ByRef @Const Mat mat);

        @ByVal
        public static native MatExpr eye(int i, int i2, int i3);

        @ByVal
        public static native MatExpr eye(@ByVal Size size, int i);

        public static native MatAllocator getDefaultAllocator();

        public static native MatAllocator getStdAllocator();

        @ByVal
        public static native MatExpr ones(int i, int i2, int i3);

        @ByVal
        public static native MatExpr ones(@ByVal Size size, int i);

        public static native void setDefaultAllocator(MatAllocator matAllocator);

        @ByVal
        public static native MatExpr zeros(int i, int i2, int i3);

        @ByVal
        public static native MatExpr zeros(@ByVal Size size, int i);

        @Name({"deallocate"})
        public native void _deallocate();

        public native void addref();

        @ByRef
        public native Mat adjustROI(int i, int i2, int i3, int i4);

        public native Mat allocator(MatAllocator matAllocator);

        public native MatAllocator allocator();

        @ByVal
        @Name({"operator ()"})
        public native Mat apply(@Const Range range);

        @ByVal
        @Name({"operator ()"})
        public native Mat apply(@ByVal Range range, @ByVal Range range2);

        @ByVal
        @Name({"operator ()"})
        public native Mat apply(@ByRef @Const Rect rect);

        public native void assignTo(@ByRef Mat mat);

        public native void assignTo(@ByRef Mat mat, int i);

        public native int channels();

        public native int checkVector(int i);

        public native int checkVector(int i, int i2, @Cast({"bool"}) boolean z);

        @ByVal
        public native Mat clone();

        @ByVal
        public native Mat col(int i);

        @ByVal
        public native Mat colRange(int i, int i2);

        @ByVal
        public native Mat colRange(@ByRef @Const Range range);

        public native int cols();

        public native Mat cols(int i);

        public native void convertTo(@ByVal GpuMat gpuMat, int i);

        public native void convertTo(@ByVal GpuMat gpuMat, int i, double d, double d2);

        public native void convertTo(@ByVal Mat mat, int i);

        public native void convertTo(@ByVal Mat mat, int i, double d, double d2);

        public native void convertTo(@ByVal UMat uMat, int i);

        public native void convertTo(@ByVal UMat uMat, int i, double d, double d2);

        public native void copySize(@ByRef @Const Mat mat);

        public native void copyTo(@ByVal GpuMat gpuMat);

        public native void copyTo(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

        public native void copyTo(@ByVal Mat mat);

        public native void copyTo(@ByVal Mat mat, @ByVal Mat mat2);

        public native void copyTo(@ByVal UMat uMat);

        public native void copyTo(@ByVal UMat uMat, @ByVal UMat uMat2);

        public native void create(int i, int i2, int i3);

        public native void create(int i, @Const IntBuffer intBuffer, int i2);

        public native void create(int i, @Const IntPointer intPointer, int i2);

        public native void create(int i, @Const int[] iArr, int i2);

        public native void create(@StdVector IntBuffer intBuffer, int i);

        public native void create(@StdVector IntPointer intPointer, int i);

        public native void create(@ByVal Size size, int i);

        public native void create(@StdVector int[] iArr, int i);

        @ByVal
        public native Mat cross(@ByVal GpuMat gpuMat);

        @ByVal
        public native Mat cross(@ByVal Mat mat);

        @ByVal
        public native Mat cross(@ByVal UMat uMat);

        @Cast({"uchar*"})
        public native BytePointer data();

        public native Mat data(BytePointer bytePointer);

        @MemberGetter
        @Cast({"const uchar*"})
        public native BytePointer dataend();

        @MemberGetter
        @Cast({"const uchar*"})
        public native BytePointer datalimit();

        @MemberGetter
        @Cast({"const uchar*"})
        public native BytePointer datastart();

        public native int depth();

        @ByVal
        public native Mat diag();

        @ByVal
        public native Mat diag(int i);

        public native int dims();

        public native Mat dims(int i);

        public native double dot(@ByVal GpuMat gpuMat);

        public native double dot(@ByVal Mat mat);

        public native double dot(@ByVal UMat uMat);

        @Cast({"size_t"})
        public native long elemSize();

        @Cast({"size_t"})
        public native long elemSize1();

        @Cast({"bool"})
        public native boolean empty();

        public native int flags();

        public native Mat flags(int i);

        @ByVal
        public native UMat getUMat(@Cast({"cv::AccessFlag"}) int i);

        @ByVal
        public native UMat getUMat(@Cast({"cv::AccessFlag"}) int i, @Cast({"cv::UMatUsageFlags"}) int i2);

        @ByVal
        public native MatExpr inv();

        @ByVal
        public native MatExpr inv(int i);

        @Cast({"bool"})
        public native boolean isContinuous();

        @Cast({"bool"})
        public native boolean isSubmatrix();

        public native void locateROI(@ByRef Size size, @ByRef Point point);

        @ByVal
        public native MatExpr mul(@ByVal GpuMat gpuMat);

        @ByVal
        public native MatExpr mul(@ByVal GpuMat gpuMat, double d);

        @ByVal
        public native MatExpr mul(@ByVal Mat mat);

        @ByVal
        public native MatExpr mul(@ByVal Mat mat, double d);

        @ByVal
        public native MatExpr mul(@ByVal UMat uMat);

        @ByVal
        public native MatExpr mul(@ByVal UMat uMat, double d);

        public native void pop_back();

        public native void pop_back(@Cast({"size_t"}) long j);

        @Cast({"uchar*"})
        public native ByteBuffer ptr(@Const IntBuffer intBuffer);

        @Cast({"uchar*"})
        public native BytePointer ptr();

        @Cast({"uchar*"})
        public native BytePointer ptr(int i);

        @Cast({"uchar*"})
        public native BytePointer ptr(int i, int i2);

        @Cast({"uchar*"})
        public native BytePointer ptr(int i, int i2, int i3);

        @Cast({"uchar*"})
        public native BytePointer ptr(@Const IntPointer intPointer);

        @Cast({"uchar*"})
        public native byte[] ptr(@Const int[] iArr);

        public native void push_back(@ByRef @Const Mat mat);

        public native void push_back_(@Const Pointer pointer2);

        @ByRef
        @Name({"operator ="})
        public native Mat put(@ByRef @Const Mat mat);

        @ByRef
        @Name({"operator ="})
        public native Mat put(@ByRef @Const MatExpr matExpr);

        @ByRef
        @Name({"operator ="})
        public native Mat put(@ByRef @Const Scalar scalar);

        public native void release();

        public native void reserve(@Cast({"size_t"}) long j);

        public native void reserveBuffer(@Cast({"size_t"}) long j);

        @ByVal
        public native Mat reshape(int i);

        @ByVal
        public native Mat reshape(int i, int i2);

        @ByVal
        public native Mat reshape(int i, int i2, @Const IntBuffer intBuffer);

        @ByVal
        public native Mat reshape(int i, int i2, @Const IntPointer intPointer);

        @ByVal
        public native Mat reshape(int i, int i2, @Const int[] iArr);

        @ByVal
        public native Mat reshape(int i, @StdVector IntBuffer intBuffer);

        @ByVal
        public native Mat reshape(int i, @StdVector IntPointer intPointer);

        @ByVal
        public native Mat reshape(int i, @StdVector int[] iArr);

        public native void resize(@Cast({"size_t"}) long j);

        public native void resize(@Cast({"size_t"}) long j, @ByRef @Const Scalar scalar);

        @ByVal
        public native Mat row(int i);

        @ByVal
        public native Mat rowRange(int i, int i2);

        @ByVal
        public native Mat rowRange(@ByRef @Const Range range);

        public native int rows();

        public native Mat rows(int i);

        @ByRef
        public native Mat setTo(@ByVal GpuMat gpuMat);

        @ByRef
        public native Mat setTo(@ByVal GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

        @ByRef
        public native Mat setTo(@ByVal Mat mat);

        @ByRef
        public native Mat setTo(@ByVal Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

        @ByRef
        public native Mat setTo(@ByVal UMat uMat);

        @ByRef
        public native Mat setTo(@ByVal UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

        @MemberGetter
        public native int size(int i);

        @ByVal
        public native Size size();

        @MemberGetter
        public native int step(int i);

        @MemberGetter
        public native long step();

        @Cast({"size_t"})
        public native long step1();

        @Cast({"size_t"})
        public native long step1(int i);

        @ByVal
        public native MatExpr t();

        @Cast({"size_t"})
        public native long total();

        @Cast({"size_t"})
        public native long total(int i);

        @Cast({"size_t"})
        public native long total(int i, int i2);

        public native int type();

        public native Mat u(UMatData uMatData);

        public native UMatData u();

        public native void updateContinuityFlag();

        static {
            Loader.load();
        }

        public Mat(Pointer p) {
            super(p);
        }

        public Mat(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Mat position(long position) {
            return (Mat) super.position(position);
        }

        public Mat() {
            super((Pointer) null);
            allocate();
        }

        public Mat(int rows, int cols, int type) {
            super((Pointer) null);
            allocate(rows, cols, type);
        }

        public Mat(@ByVal Size size, int type) {
            super((Pointer) null);
            allocate(size, type);
        }

        public Mat(int rows, int cols, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(rows, cols, type, s);
        }

        public Mat(@ByVal Size size, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(size, type, s);
        }

        public Mat(int ndims, @Const IntPointer sizes, int type) {
            super((Pointer) null);
            allocate(ndims, sizes, type);
        }

        public Mat(int ndims, @Const IntBuffer sizes, int type) {
            super((Pointer) null);
            allocate(ndims, sizes, type);
        }

        public Mat(int ndims, @Const int[] sizes, int type) {
            super((Pointer) null);
            allocate(ndims, sizes, type);
        }

        public Mat(@StdVector IntPointer sizes, int type) {
            super((Pointer) null);
            allocate(sizes, type);
        }

        public Mat(@StdVector IntBuffer sizes, int type) {
            super((Pointer) null);
            allocate(sizes, type);
        }

        public Mat(@StdVector int[] sizes, int type) {
            super((Pointer) null);
            allocate(sizes, type);
        }

        public Mat(int ndims, @Const IntPointer sizes, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(ndims, sizes, type, s);
        }

        public Mat(int ndims, @Const IntBuffer sizes, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(ndims, sizes, type, s);
        }

        public Mat(int ndims, @Const int[] sizes, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(ndims, sizes, type, s);
        }

        public Mat(@StdVector IntPointer sizes, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(sizes, type, s);
        }

        public Mat(@StdVector IntBuffer sizes, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(sizes, type, s);
        }

        public Mat(@StdVector int[] sizes, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(sizes, type, s);
        }

        public Mat(@ByRef @Const Mat m) {
            super((Pointer) null);
            allocate(m);
        }

        public Mat(int rows, int cols, int type, Pointer data, @Cast({"size_t"}) long step) {
            super((Pointer) null);
            allocate(rows, cols, type, data, step);
            this.pointer = data;
        }

        public Mat(int rows, int cols, int type, Pointer data) {
            this(rows, cols, type, data, 0);
        }

        public Mat(opencv_core.CvArr arr) {
            super(opencv_core.cvarrToMat(arr));
            this.pointer = arr;
        }

        public Mat(Point points) {
            this(1, Math.max(1, points.limit - points.position), opencv_core.CV_32SC2, (Pointer) points);
            this.pointer = points;
        }

        public Mat(Point2f points) {
            this(1, Math.max(1, points.limit - points.position), opencv_core.CV_32FC2, (Pointer) points);
            this.pointer = points;
        }

        public Mat(Point2d points) {
            this(1, Math.max(1, points.limit - points.position), opencv_core.CV_64FC2, (Pointer) points);
            this.pointer = points;
        }

        public Mat(Point3i points) {
            this(1, Math.max(1, points.limit - points.position), opencv_core.CV_32SC3, (Pointer) points);
            this.pointer = points;
        }

        public Mat(Point3f points) {
            this(1, Math.max(1, points.limit - points.position), opencv_core.CV_32FC3, (Pointer) points);
            this.pointer = points;
        }

        public Mat(Point3d points) {
            this(1, Math.max(1, points.limit - points.position), opencv_core.CV_64FC3, (Pointer) points);
            this.pointer = points;
        }

        public Mat(Scalar scalar) {
            this(1, Math.max(1, scalar.limit - scalar.position), opencv_core.CV_64FC4, (Pointer) scalar);
            this.pointer = scalar;
        }

        public Mat(Scalar4i scalar) {
            this(1, Math.max(1, scalar.limit - scalar.position), opencv_core.CV_32SC4, (Pointer) scalar);
            this.pointer = scalar;
        }

        public Mat(byte... b) {
            this(b, false);
        }

        public Mat(byte[] b, boolean signed) {
            this(new BytePointer(b), signed);
        }

        public Mat(short... s) {
            this(s, true);
        }

        public Mat(short[] s, boolean signed) {
            this(new ShortPointer(s), signed);
        }

        public Mat(int... n) {
            this(new IntPointer(n));
        }

        public Mat(double... d) {
            this(new DoublePointer(d));
        }

        public Mat(float... f) {
            this(new FloatPointer(f));
        }

        /* JADX INFO: this call moved to the top of the method (can break code semantics) */
        private Mat(long rows, long cols, int type, Pointer data) {
            this((int) Math.min(rows, TTL.MAX_VALUE), (int) Math.min(cols, TTL.MAX_VALUE), type, data, 0);
            long j = rows;
        }

        public Mat(BytePointer p) {
            this(p, false);
        }

        /* JADX INFO: this call moved to the top of the method (can break code semantics) */
        public Mat(BytePointer p, boolean signed) {
            this(1, Math.max(1, p.limit - p.position), signed ? opencv_core.CV_8SC1 : opencv_core.CV_8UC1, (Pointer) p);
        }

        public Mat(ShortPointer p) {
            this(p, false);
        }

        /* JADX INFO: this call moved to the top of the method (can break code semantics) */
        public Mat(ShortPointer p, boolean signed) {
            this(1, Math.max(1, p.limit - p.position), signed ? opencv_core.CV_16SC1 : opencv_core.CV_16UC1, (Pointer) p);
        }

        public Mat(IntPointer p) {
            this(1, Math.max(1, p.limit - p.position), opencv_core.CV_32SC1, (Pointer) p);
        }

        public Mat(FloatPointer p) {
            this(1, Math.max(1, p.limit - p.position), opencv_core.CV_32FC1, (Pointer) p);
        }

        public Mat(DoublePointer p) {
            this(1, Math.max(1, p.limit - p.position), opencv_core.CV_64FC1, (Pointer) p);
        }

        public Mat(@ByVal Size size, int type, Pointer data, @Cast({"size_t"}) long step) {
            super((Pointer) null);
            allocate(size, type, data, step);
        }

        public Mat(@ByVal Size size, int type, Pointer data) {
            super((Pointer) null);
            allocate(size, type, data);
        }

        public Mat(int ndims, @Const IntPointer sizes, int type, Pointer data, @Cast({"const size_t*"}) SizeTPointer steps) {
            super((Pointer) null);
            allocate(ndims, sizes, type, data, steps);
        }

        public Mat(int ndims, @Const IntPointer sizes, int type, Pointer data) {
            super((Pointer) null);
            allocate(ndims, sizes, type, data);
        }

        public Mat(int ndims, @Const IntBuffer sizes, int type, Pointer data, @Cast({"const size_t*"}) SizeTPointer steps) {
            super((Pointer) null);
            allocate(ndims, sizes, type, data, steps);
        }

        public Mat(int ndims, @Const IntBuffer sizes, int type, Pointer data) {
            super((Pointer) null);
            allocate(ndims, sizes, type, data);
        }

        public Mat(int ndims, @Const int[] sizes, int type, Pointer data, @Cast({"const size_t*"}) SizeTPointer steps) {
            super((Pointer) null);
            allocate(ndims, sizes, type, data, steps);
        }

        public Mat(int ndims, @Const int[] sizes, int type, Pointer data) {
            super((Pointer) null);
            allocate(ndims, sizes, type, data);
        }

        public Mat(@StdVector IntPointer sizes, int type, Pointer data, @Cast({"const size_t*"}) SizeTPointer steps) {
            super((Pointer) null);
            allocate(sizes, type, data, steps);
        }

        public Mat(@StdVector IntPointer sizes, int type, Pointer data) {
            super((Pointer) null);
            allocate(sizes, type, data);
        }

        public Mat(@StdVector IntBuffer sizes, int type, Pointer data, @Cast({"const size_t*"}) SizeTPointer steps) {
            super((Pointer) null);
            allocate(sizes, type, data, steps);
        }

        public Mat(@StdVector IntBuffer sizes, int type, Pointer data) {
            super((Pointer) null);
            allocate(sizes, type, data);
        }

        public Mat(@StdVector int[] sizes, int type, Pointer data, @Cast({"const size_t*"}) SizeTPointer steps) {
            super((Pointer) null);
            allocate(sizes, type, data, steps);
        }

        public Mat(@StdVector int[] sizes, int type, Pointer data) {
            super((Pointer) null);
            allocate(sizes, type, data);
        }

        public Mat(@ByRef @Const Mat m, @ByRef @Const Range rowRange, @ByRef(nullValue = "cv::Range::all()") @Const Range colRange) {
            super((Pointer) null);
            allocate(m, rowRange, colRange);
        }

        public Mat(@ByRef @Const Mat m, @ByRef @Const Range rowRange) {
            super((Pointer) null);
            allocate(m, rowRange);
        }

        public Mat(@ByRef @Const Mat m, @ByRef @Const Rect roi) {
            super((Pointer) null);
            allocate(m, roi);
        }

        public Mat(@ByRef @Const GpuMat m) {
            super((Pointer) null);
            allocate(m);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class UMat extends Pointer {
        public static final int AUTO_STEP = 0;
        public static final int CONTINUOUS_FLAG = 16384;
        public static final int DEPTH_MASK = 7;
        public static final int MAGIC_MASK = -65536;
        public static final int MAGIC_VAL = 1124007936;
        public static final int SUBMATRIX_FLAG = 32768;
        public static final int TYPE_MASK = 4095;

        private native void allocate();

        private native void allocate(@Cast({"cv::UMatUsageFlags"}) int i);

        private native void allocate(int i, int i2, int i3);

        private native void allocate(int i, int i2, int i3, @Cast({"cv::UMatUsageFlags"}) int i4);

        private native void allocate(int i, int i2, int i3, @ByRef @Const Scalar scalar);

        private native void allocate(int i, int i2, int i3, @ByRef @Const Scalar scalar, @Cast({"cv::UMatUsageFlags"}) int i4);

        private native void allocate(int i, @Const IntBuffer intBuffer, int i2);

        private native void allocate(int i, @Const IntBuffer intBuffer, int i2, @Cast({"cv::UMatUsageFlags"}) int i3);

        private native void allocate(int i, @Const IntBuffer intBuffer, int i2, @ByRef @Const Scalar scalar);

        private native void allocate(int i, @Const IntBuffer intBuffer, int i2, @ByRef @Const Scalar scalar, @Cast({"cv::UMatUsageFlags"}) int i3);

        private native void allocate(int i, @Const IntPointer intPointer, int i2);

        private native void allocate(int i, @Const IntPointer intPointer, int i2, @Cast({"cv::UMatUsageFlags"}) int i3);

        private native void allocate(int i, @Const IntPointer intPointer, int i2, @ByRef @Const Scalar scalar);

        private native void allocate(int i, @Const IntPointer intPointer, int i2, @ByRef @Const Scalar scalar, @Cast({"cv::UMatUsageFlags"}) int i3);

        private native void allocate(int i, @Const int[] iArr, int i2);

        private native void allocate(int i, @Const int[] iArr, int i2, @Cast({"cv::UMatUsageFlags"}) int i3);

        private native void allocate(int i, @Const int[] iArr, int i2, @ByRef @Const Scalar scalar);

        private native void allocate(int i, @Const int[] iArr, int i2, @ByRef @Const Scalar scalar, @Cast({"cv::UMatUsageFlags"}) int i3);

        private native void allocate(@ByVal Size size, int i);

        private native void allocate(@ByVal Size size, int i, @Cast({"cv::UMatUsageFlags"}) int i2);

        private native void allocate(@ByVal Size size, int i, @ByRef @Const Scalar scalar);

        private native void allocate(@ByVal Size size, int i, @ByRef @Const Scalar scalar, @Cast({"cv::UMatUsageFlags"}) int i2);

        private native void allocate(@ByRef @Const UMat uMat);

        private native void allocate(@ByRef @Const UMat uMat, @ByRef @Const Range range);

        private native void allocate(@ByRef @Const UMat uMat, @ByRef @Const Range range, @ByRef(nullValue = "cv::Range::all()") @Const Range range2);

        private native void allocate(@ByRef @Const UMat uMat, @ByRef @Const Rect rect);

        private native void allocateArray(long j);

        @ByVal
        public static native UMat diag(@ByRef @Const UMat uMat);

        @ByVal
        public static native UMat eye(int i, int i2, int i3);

        @ByVal
        public static native UMat eye(@ByVal Size size, int i);

        public static native MatAllocator getStdAllocator();

        @ByVal
        public static native UMat ones(int i, int i2, int i3);

        @ByVal
        public static native UMat ones(@ByVal Size size, int i);

        @ByVal
        public static native UMat zeros(int i, int i2, int i3);

        @ByVal
        public static native UMat zeros(@ByVal Size size, int i);

        @Name({"deallocate"})
        public native void _deallocate();

        public native void addref();

        @ByRef
        public native UMat adjustROI(int i, int i2, int i3, int i4);

        public native MatAllocator allocator();

        public native UMat allocator(MatAllocator matAllocator);

        @ByVal
        @Name({"operator ()"})
        public native UMat apply(@Const Range range);

        @ByVal
        @Name({"operator ()"})
        public native UMat apply(@ByVal Range range, @ByVal Range range2);

        @ByVal
        @Name({"operator ()"})
        public native UMat apply(@ByRef @Const Rect rect);

        public native void assignTo(@ByRef UMat uMat);

        public native void assignTo(@ByRef UMat uMat, int i);

        public native int channels();

        public native int checkVector(int i);

        public native int checkVector(int i, int i2, @Cast({"bool"}) boolean z);

        @ByVal
        public native UMat clone();

        @ByVal
        public native UMat col(int i);

        @ByVal
        public native UMat colRange(int i, int i2);

        @ByVal
        public native UMat colRange(@ByRef @Const Range range);

        public native int cols();

        public native UMat cols(int i);

        public native void convertTo(@ByVal GpuMat gpuMat, int i);

        public native void convertTo(@ByVal GpuMat gpuMat, int i, double d, double d2);

        public native void convertTo(@ByVal Mat mat, int i);

        public native void convertTo(@ByVal Mat mat, int i, double d, double d2);

        public native void convertTo(@ByVal UMat uMat, int i);

        public native void convertTo(@ByVal UMat uMat, int i, double d, double d2);

        public native void copySize(@ByRef @Const UMat uMat);

        public native void copyTo(@ByVal GpuMat gpuMat);

        public native void copyTo(@ByVal GpuMat gpuMat, @ByVal GpuMat gpuMat2);

        public native void copyTo(@ByVal Mat mat);

        public native void copyTo(@ByVal Mat mat, @ByVal Mat mat2);

        public native void copyTo(@ByVal UMat uMat);

        public native void copyTo(@ByVal UMat uMat, @ByVal UMat uMat2);

        public native void create(int i, int i2, int i3);

        public native void create(int i, int i2, int i3, @Cast({"cv::UMatUsageFlags"}) int i4);

        public native void create(int i, @Const IntBuffer intBuffer, int i2);

        public native void create(int i, @Const IntBuffer intBuffer, int i2, @Cast({"cv::UMatUsageFlags"}) int i3);

        public native void create(int i, @Const IntPointer intPointer, int i2);

        public native void create(int i, @Const IntPointer intPointer, int i2, @Cast({"cv::UMatUsageFlags"}) int i3);

        public native void create(int i, @Const int[] iArr, int i2);

        public native void create(int i, @Const int[] iArr, int i2, @Cast({"cv::UMatUsageFlags"}) int i3);

        public native void create(@StdVector IntBuffer intBuffer, int i);

        public native void create(@StdVector IntBuffer intBuffer, int i, @Cast({"cv::UMatUsageFlags"}) int i2);

        public native void create(@StdVector IntPointer intPointer, int i);

        public native void create(@StdVector IntPointer intPointer, int i, @Cast({"cv::UMatUsageFlags"}) int i2);

        public native void create(@ByVal Size size, int i);

        public native void create(@ByVal Size size, int i, @Cast({"cv::UMatUsageFlags"}) int i2);

        public native void create(@StdVector int[] iArr, int i);

        public native void create(@StdVector int[] iArr, int i, @Cast({"cv::UMatUsageFlags"}) int i2);

        public native int depth();

        @ByVal
        public native UMat diag();

        @ByVal
        public native UMat diag(int i);

        public native int dims();

        public native UMat dims(int i);

        public native double dot(@ByVal GpuMat gpuMat);

        public native double dot(@ByVal Mat mat);

        public native double dot(@ByVal UMat uMat);

        @Cast({"size_t"})
        public native long elemSize();

        @Cast({"size_t"})
        public native long elemSize1();

        @Cast({"bool"})
        public native boolean empty();

        public native int flags();

        public native UMat flags(int i);

        @ByVal
        public native Mat getMat(@Cast({"cv::AccessFlag"}) int i);

        public native Pointer handle(@Cast({"cv::AccessFlag"}) int i);

        @ByVal
        public native UMat inv();

        @ByVal
        public native UMat inv(int i);

        @Cast({"bool"})
        public native boolean isContinuous();

        @Cast({"bool"})
        public native boolean isSubmatrix();

        public native void locateROI(@ByRef Size size, @ByRef Point point);

        @ByVal
        public native UMat mul(@ByVal GpuMat gpuMat);

        @ByVal
        public native UMat mul(@ByVal GpuMat gpuMat, double d);

        @ByVal
        public native UMat mul(@ByVal Mat mat);

        @ByVal
        public native UMat mul(@ByVal Mat mat, double d);

        @ByVal
        public native UMat mul(@ByVal UMat uMat);

        @ByVal
        public native UMat mul(@ByVal UMat uMat, double d);

        public native void ndoffset(@Cast({"size_t*"}) SizeTPointer sizeTPointer);

        @Cast({"size_t"})
        public native long offset();

        public native UMat offset(long j);

        @ByRef
        @Name({"operator ="})
        public native UMat put(@ByRef @Const Scalar scalar);

        @ByRef
        @Name({"operator ="})
        public native UMat put(@ByRef @Const UMat uMat);

        public native void release();

        @ByVal
        public native UMat reshape(int i);

        @ByVal
        public native UMat reshape(int i, int i2);

        @ByVal
        public native UMat reshape(int i, int i2, @Const IntBuffer intBuffer);

        @ByVal
        public native UMat reshape(int i, int i2, @Const IntPointer intPointer);

        @ByVal
        public native UMat reshape(int i, int i2, @Const int[] iArr);

        @ByVal
        public native UMat row(int i);

        @ByVal
        public native UMat rowRange(int i, int i2);

        @ByVal
        public native UMat rowRange(@ByRef @Const Range range);

        public native int rows();

        public native UMat rows(int i);

        @ByRef
        public native UMat setTo(@ByVal GpuMat gpuMat);

        @ByRef
        public native UMat setTo(@ByVal GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") GpuMat gpuMat2);

        @ByRef
        public native UMat setTo(@ByVal Mat mat);

        @ByRef
        public native UMat setTo(@ByVal Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") Mat mat2);

        @ByRef
        public native UMat setTo(@ByVal UMat uMat);

        @ByRef
        public native UMat setTo(@ByVal UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") UMat uMat2);

        @MemberGetter
        public native int size(int i);

        @ByVal
        public native Size size();

        @MemberGetter
        public native int step(int i);

        @MemberGetter
        public native long step();

        @Cast({"size_t"})
        public native long step1();

        @Cast({"size_t"})
        public native long step1(int i);

        @ByVal
        public native UMat t();

        @Cast({"size_t"})
        public native long total();

        public native int type();

        public native UMat u(UMatData uMatData);

        public native UMatData u();

        public native void updateContinuityFlag();

        @Cast({"cv::UMatUsageFlags"})
        public native int usageFlags();

        public native UMat usageFlags(int i);

        static {
            Loader.load();
        }

        public UMat(Pointer p) {
            super(p);
        }

        public UMat(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public UMat position(long position) {
            return (UMat) super.position(position);
        }

        public UMat(@Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(usageFlags);
        }

        public UMat() {
            super((Pointer) null);
            allocate();
        }

        public UMat(int rows, int cols, int type, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(rows, cols, type, usageFlags);
        }

        public UMat(int rows, int cols, int type) {
            super((Pointer) null);
            allocate(rows, cols, type);
        }

        public UMat(@ByVal Size size, int type, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(size, type, usageFlags);
        }

        public UMat(@ByVal Size size, int type) {
            super((Pointer) null);
            allocate(size, type);
        }

        public UMat(int rows, int cols, int type, @ByRef @Const Scalar s, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(rows, cols, type, s, usageFlags);
        }

        public UMat(int rows, int cols, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(rows, cols, type, s);
        }

        public UMat(@ByVal Size size, int type, @ByRef @Const Scalar s, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(size, type, s, usageFlags);
        }

        public UMat(@ByVal Size size, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(size, type, s);
        }

        public UMat(int ndims, @Const IntPointer sizes, int type, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(ndims, sizes, type, usageFlags);
        }

        public UMat(int ndims, @Const IntPointer sizes, int type) {
            super((Pointer) null);
            allocate(ndims, sizes, type);
        }

        public UMat(int ndims, @Const IntBuffer sizes, int type, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(ndims, sizes, type, usageFlags);
        }

        public UMat(int ndims, @Const IntBuffer sizes, int type) {
            super((Pointer) null);
            allocate(ndims, sizes, type);
        }

        public UMat(int ndims, @Const int[] sizes, int type, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(ndims, sizes, type, usageFlags);
        }

        public UMat(int ndims, @Const int[] sizes, int type) {
            super((Pointer) null);
            allocate(ndims, sizes, type);
        }

        public UMat(int ndims, @Const IntPointer sizes, int type, @ByRef @Const Scalar s, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(ndims, sizes, type, s, usageFlags);
        }

        public UMat(int ndims, @Const IntPointer sizes, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(ndims, sizes, type, s);
        }

        public UMat(int ndims, @Const IntBuffer sizes, int type, @ByRef @Const Scalar s, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(ndims, sizes, type, s, usageFlags);
        }

        public UMat(int ndims, @Const IntBuffer sizes, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(ndims, sizes, type, s);
        }

        public UMat(int ndims, @Const int[] sizes, int type, @ByRef @Const Scalar s, @Cast({"cv::UMatUsageFlags"}) int usageFlags) {
            super((Pointer) null);
            allocate(ndims, sizes, type, s, usageFlags);
        }

        public UMat(int ndims, @Const int[] sizes, int type, @ByRef @Const Scalar s) {
            super((Pointer) null);
            allocate(ndims, sizes, type, s);
        }

        public UMat(@ByRef @Const UMat m) {
            super((Pointer) null);
            allocate(m);
        }

        public UMat(@ByRef @Const UMat m, @ByRef @Const Range rowRange, @ByRef(nullValue = "cv::Range::all()") @Const Range colRange) {
            super((Pointer) null);
            allocate(m, rowRange, colRange);
        }

        public UMat(@ByRef @Const UMat m, @ByRef @Const Range rowRange) {
            super((Pointer) null);
            allocate(m, rowRange);
        }

        public UMat(@ByRef @Const UMat m, @ByRef @Const Rect roi) {
            super((Pointer) null);
            allocate(m, roi);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class SparseMat extends Pointer {
        public static final int HASH_BIT = Integer.MIN_VALUE;
        public static final int HASH_SCALE = 1540483477;
        public static final int MAGIC_VAL = 1123876864;
        public static final int MAX_DIM = 32;

        private native void allocate();

        private native void allocate(int i, @Const IntBuffer intBuffer, int i2);

        private native void allocate(int i, @Const IntPointer intPointer, int i2);

        private native void allocate(int i, @Const int[] iArr, int i2);

        private native void allocate(@ByRef @Const Mat mat);

        private native void allocate(@ByRef @Const SparseMat sparseMat);

        private native void allocateArray(long j);

        public native void addref();

        public native void assignTo(@ByRef SparseMat sparseMat);

        public native void assignTo(@ByRef SparseMat sparseMat, int i);

        @ByVal
        public native SparseMatIterator begin();

        public native int channels();

        public native void clear();

        @ByVal
        public native SparseMat clone();

        public native void convertTo(@ByRef Mat mat, int i);

        public native void convertTo(@ByRef Mat mat, int i, double d, double d2);

        public native void convertTo(@ByRef SparseMat sparseMat, int i);

        public native void convertTo(@ByRef SparseMat sparseMat, int i, double d);

        public native void copyTo(@ByRef Mat mat);

        public native void copyTo(@ByRef SparseMat sparseMat);

        public native void create(int i, @Const IntBuffer intBuffer, int i2);

        public native void create(int i, @Const IntPointer intPointer, int i2);

        public native void create(int i, @Const int[] iArr, int i2);

        public native int depth();

        public native int dims();

        @Cast({"size_t"})
        public native long elemSize();

        @Cast({"size_t"})
        public native long elemSize1();

        @ByVal
        public native SparseMatIterator end();

        public native void erase(int i, int i2);

        public native void erase(int i, int i2, int i3);

        public native void erase(int i, int i2, int i3, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        public native void erase(int i, int i2, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        public native void erase(@Const IntBuffer intBuffer);

        public native void erase(@Const IntBuffer intBuffer, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        public native void erase(@Const IntPointer intPointer);

        public native void erase(@Const IntPointer intPointer, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        public native void erase(@Const int[] iArr);

        public native void erase(@Const int[] iArr, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        public native int flags();

        public native SparseMat flags(int i);

        @Cast({"size_t"})
        public native long hash(int i);

        @Cast({"size_t"})
        public native long hash(int i, int i2);

        @Cast({"size_t"})
        public native long hash(int i, int i2, int i3);

        @Cast({"size_t"})
        public native long hash(@Const IntBuffer intBuffer);

        @Cast({"size_t"})
        public native long hash(@Const IntPointer intPointer);

        @Cast({"size_t"})
        public native long hash(@Const int[] iArr);

        public native Hdr hdr();

        public native SparseMat hdr(Hdr hdr);

        @Cast({"uchar*"})
        public native ByteBuffer newNode(@Const IntBuffer intBuffer, @Cast({"size_t"}) long j);

        @Cast({"uchar*"})
        public native BytePointer newNode(@Const IntPointer intPointer, @Cast({"size_t"}) long j);

        @Cast({"uchar*"})
        public native byte[] newNode(@Const int[] iArr, @Cast({"size_t"}) long j);

        public native Node node(@Cast({"size_t"}) long j);

        @Cast({"size_t"})
        public native long nzcount();

        @Cast({"uchar*"})
        public native ByteBuffer ptr(@Const IntBuffer intBuffer, @Cast({"bool"}) boolean z);

        @Cast({"uchar*"})
        public native ByteBuffer ptr(@Const IntBuffer intBuffer, @Cast({"bool"}) boolean z, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        @Cast({"uchar*"})
        public native BytePointer ptr(int i, int i2, int i3, @Cast({"bool"}) boolean z);

        @Cast({"uchar*"})
        public native BytePointer ptr(int i, int i2, int i3, @Cast({"bool"}) boolean z, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        @Cast({"uchar*"})
        public native BytePointer ptr(int i, int i2, @Cast({"bool"}) boolean z);

        @Cast({"uchar*"})
        public native BytePointer ptr(int i, int i2, @Cast({"bool"}) boolean z, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        @Cast({"uchar*"})
        public native BytePointer ptr(int i, @Cast({"bool"}) boolean z);

        @Cast({"uchar*"})
        public native BytePointer ptr(int i, @Cast({"bool"}) boolean z, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        @Cast({"uchar*"})
        public native BytePointer ptr(@Const IntPointer intPointer, @Cast({"bool"}) boolean z);

        @Cast({"uchar*"})
        public native BytePointer ptr(@Const IntPointer intPointer, @Cast({"bool"}) boolean z, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        @Cast({"uchar*"})
        public native byte[] ptr(@Const int[] iArr, @Cast({"bool"}) boolean z);

        @Cast({"uchar*"})
        public native byte[] ptr(@Const int[] iArr, @Cast({"bool"}) boolean z, @Cast({"size_t*"}) SizeTPointer sizeTPointer);

        @ByRef
        @Name({"operator ="})
        public native SparseMat put(@ByRef @Const Mat mat);

        @ByRef
        @Name({"operator ="})
        public native SparseMat put(@ByRef @Const SparseMat sparseMat);

        public native void release();

        public native void removeNode(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2, @Cast({"size_t"}) long j3);

        public native void resizeHashTab(@Cast({"size_t"}) long j);

        public native int size(int i);

        @Const
        public native IntPointer size();

        public native int type();

        static {
            Loader.load();
        }

        public SparseMat(Pointer p) {
            super(p);
        }

        public SparseMat(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public SparseMat position(long position) {
            return (SparseMat) super.position(position);
        }

        @NoOffset
        public static class Hdr extends Pointer {
            private native void allocate(int i, @Const IntBuffer intBuffer, int i2);

            private native void allocate(int i, @Const IntPointer intPointer, int i2);

            private native void allocate(int i, @Const int[] iArr, int i2);

            public native void clear();

            public native int dims();

            public native Hdr dims(int i);

            @Cast({"size_t"})
            public native long freeList();

            public native Hdr freeList(long j);

            @Cast({"size_t*"})
            @StdVector
            public native SizeTPointer hashtab();

            public native Hdr hashtab(SizeTPointer sizeTPointer);

            @Cast({"size_t"})
            public native long nodeCount();

            public native Hdr nodeCount(long j);

            @Cast({"size_t"})
            public native long nodeSize();

            public native Hdr nodeSize(long j);

            @Cast({"uchar*"})
            @StdVector
            public native BytePointer pool();

            public native Hdr pool(BytePointer bytePointer);

            public native int refcount();

            public native Hdr refcount(int i);

            public native int size(int i);

            @MemberGetter
            public native IntPointer size();

            public native Hdr size(int i, int i2);

            public native int valueOffset();

            public native Hdr valueOffset(int i);

            static {
                Loader.load();
            }

            public Hdr(Pointer p) {
                super(p);
            }

            public Hdr(int _dims, @Const IntPointer _sizes, int _type) {
                super((Pointer) null);
                allocate(_dims, _sizes, _type);
            }

            public Hdr(int _dims, @Const IntBuffer _sizes, int _type) {
                super((Pointer) null);
                allocate(_dims, _sizes, _type);
            }

            public Hdr(int _dims, @Const int[] _sizes, int _type) {
                super((Pointer) null);
                allocate(_dims, _sizes, _type);
            }
        }

        public static class Node extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            @Cast({"size_t"})
            public native long hashval();

            public native Node hashval(long j);

            public native int idx(int i);

            @MemberGetter
            public native IntPointer idx();

            public native Node idx(int i, int i2);

            @Cast({"size_t"})
            public native long next();

            public native Node next(long j);

            static {
                Loader.load();
            }

            public Node() {
                super((Pointer) null);
                allocate();
            }

            public Node(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public Node(Pointer p) {
                super(p);
            }

            public Node position(long position) {
                return (Node) super.position(position);
            }
        }

        public SparseMat() {
            super((Pointer) null);
            allocate();
        }

        public SparseMat(int dims, @Const IntPointer _sizes, int _type) {
            super((Pointer) null);
            allocate(dims, _sizes, _type);
        }

        public SparseMat(int dims, @Const IntBuffer _sizes, int _type) {
            super((Pointer) null);
            allocate(dims, _sizes, _type);
        }

        public SparseMat(int dims, @Const int[] _sizes, int _type) {
            super((Pointer) null);
            allocate(dims, _sizes, _type);
        }

        public SparseMat(@ByRef @Const SparseMat m) {
            super((Pointer) null);
            allocate(m);
        }

        public SparseMat(@ByRef @Const Mat m) {
            super((Pointer) null);
            allocate(m);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class MatConstIterator extends Pointer {
        private native void allocate();

        private native void allocate(@Const Mat mat);

        private native void allocate(@Const Mat mat, int i);

        private native void allocate(@Const Mat mat, int i, int i2);

        private native void allocate(@Const Mat mat, @ByVal Point point);

        private native void allocate(@ByRef @Const MatConstIterator matConstIterator);

        private native void allocateArray(long j);

        @ByRef
        @Name({"operator +="})
        public native MatConstIterator addPut(@Cast({"ptrdiff_t"}) long j);

        @ByRef
        @Name({"operator --"})
        public native MatConstIterator decrement();

        @ByVal
        @Name({"operator --"})
        public native MatConstIterator decrement(int i);

        @Cast({"size_t"})
        public native long elemSize();

        public native MatConstIterator elemSize(long j);

        @Cast({"const uchar*"})
        @Name({"operator []"})
        public native BytePointer get(@Cast({"ptrdiff_t"}) long j);

        @ByRef
        @Name({"operator ++"})
        public native MatConstIterator increment();

        @ByVal
        @Name({"operator ++"})
        public native MatConstIterator increment(int i);

        @Cast({"ptrdiff_t"})
        public native long lpos();

        @MemberGetter
        @Const
        public native Mat m();

        @Cast({"const uchar*"})
        @Name({"operator *"})
        public native BytePointer multiply();

        @ByVal
        public native Point pos();

        public native void pos(IntBuffer intBuffer);

        public native void pos(IntPointer intPointer);

        public native void pos(int[] iArr);

        @MemberGetter
        @Cast({"const uchar*"})
        public native BytePointer ptr();

        @ByRef
        @Name({"operator ="})
        public native MatConstIterator put(@ByRef @Const MatConstIterator matConstIterator);

        public native void seek(@Cast({"ptrdiff_t"}) long j);

        public native void seek(@Cast({"ptrdiff_t"}) long j, @Cast({"bool"}) boolean z);

        public native void seek(@Const IntBuffer intBuffer);

        public native void seek(@Const IntBuffer intBuffer, @Cast({"bool"}) boolean z);

        public native void seek(@Const IntPointer intPointer);

        public native void seek(@Const IntPointer intPointer, @Cast({"bool"}) boolean z);

        public native void seek(@Const int[] iArr);

        public native void seek(@Const int[] iArr, @Cast({"bool"}) boolean z);

        @MemberGetter
        @Cast({"const uchar*"})
        public native BytePointer sliceEnd();

        @MemberGetter
        @Cast({"const uchar*"})
        public native BytePointer sliceStart();

        @ByRef
        @Name({"operator -="})
        public native MatConstIterator subtractPut(@Cast({"ptrdiff_t"}) long j);

        static {
            Loader.load();
        }

        public MatConstIterator(Pointer p) {
            super(p);
        }

        public MatConstIterator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MatConstIterator position(long position) {
            return (MatConstIterator) super.position(position);
        }

        public MatConstIterator() {
            super((Pointer) null);
            allocate();
        }

        public MatConstIterator(@Const Mat _m) {
            super((Pointer) null);
            allocate(_m);
        }

        public MatConstIterator(@Const Mat _m, int _row, int _col) {
            super((Pointer) null);
            allocate(_m, _row, _col);
        }

        public MatConstIterator(@Const Mat _m, int _row) {
            super((Pointer) null);
            allocate(_m, _row);
        }

        public MatConstIterator(@Const Mat _m, @ByVal Point _pt) {
            super((Pointer) null);
            allocate(_m, _pt);
        }

        public MatConstIterator(@ByRef @Const MatConstIterator it) {
            super((Pointer) null);
            allocate(it);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class SparseMatConstIterator extends Pointer {
        private native void allocate();

        private native void allocate(@Const SparseMat sparseMat);

        private native void allocate(@ByRef @Const SparseMatConstIterator sparseMatConstIterator);

        private native void allocateArray(long j);

        @Cast({"size_t"})
        public native long hashidx();

        public native SparseMatConstIterator hashidx(long j);

        @ByRef
        @Name({"operator ++"})
        public native SparseMatConstIterator increment();

        @ByVal
        @Name({"operator ++"})
        public native SparseMatConstIterator increment(int i);

        @MemberGetter
        @Const
        public native SparseMat m();

        @Const
        public native SparseMat.Node node();

        @Cast({"uchar*"})
        public native BytePointer ptr();

        public native SparseMatConstIterator ptr(BytePointer bytePointer);

        @ByRef
        @Name({"operator ="})
        public native SparseMatConstIterator put(@ByRef @Const SparseMatConstIterator sparseMatConstIterator);

        public native void seekEnd();

        static {
            Loader.load();
        }

        public SparseMatConstIterator(Pointer p) {
            super(p);
        }

        public SparseMatConstIterator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public SparseMatConstIterator position(long position) {
            return (SparseMatConstIterator) super.position(position);
        }

        public SparseMatConstIterator() {
            super((Pointer) null);
            allocate();
        }

        public SparseMatConstIterator(@Const SparseMat _m) {
            super((Pointer) null);
            allocate(_m);
        }

        public SparseMatConstIterator(@ByRef @Const SparseMatConstIterator it) {
            super((Pointer) null);
            allocate(it);
        }
    }

    @Namespace("cv")
    public static class SparseMatIterator extends SparseMatConstIterator {
        private native void allocate();

        private native void allocate(SparseMat sparseMat);

        private native void allocate(@ByRef @Const SparseMatIterator sparseMatIterator);

        private native void allocateArray(long j);

        @ByRef
        @Name({"operator ++"})
        public native SparseMatIterator increment();

        @ByVal
        @Name({"operator ++"})
        public native SparseMatIterator increment(int i);

        public native SparseMat.Node node();

        @ByRef
        @Name({"operator ="})
        public native SparseMatIterator put(@ByRef @Const SparseMatIterator sparseMatIterator);

        static {
            Loader.load();
        }

        public SparseMatIterator(Pointer p) {
            super(p);
        }

        public SparseMatIterator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public SparseMatIterator position(long position) {
            return (SparseMatIterator) super.position(position);
        }

        public SparseMatIterator() {
            super((Pointer) null);
            allocate();
        }

        public SparseMatIterator(SparseMat _m) {
            super((Pointer) null);
            allocate(_m);
        }

        public SparseMatIterator(@ByRef @Const SparseMatIterator it) {
            super((Pointer) null);
            allocate(it);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class NAryMatIterator extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"const cv::Mat**"}) PointerPointer pointerPointer, @Cast({"uchar**"}) PointerPointer pointerPointer2, int i);

        private native void allocate(@Cast({"const cv::Mat**"}) PointerPointer pointerPointer, Mat mat, int i);

        private native void allocate(@ByPtrPtr @Const Mat mat, @ByPtrPtr @Cast({"uchar**"}) ByteBuffer byteBuffer);

        private native void allocate(@ByPtrPtr @Const Mat mat, @ByPtrPtr @Cast({"uchar**"}) ByteBuffer byteBuffer, int i);

        private native void allocate(@ByPtrPtr @Const Mat mat, @ByPtrPtr @Cast({"uchar**"}) BytePointer bytePointer);

        private native void allocate(@ByPtrPtr @Const Mat mat, @ByPtrPtr @Cast({"uchar**"}) BytePointer bytePointer, int i);

        private native void allocate(@ByPtrPtr @Const Mat mat, Mat mat2);

        private native void allocate(@ByPtrPtr @Const Mat mat, Mat mat2, int i);

        private native void allocate(@ByPtrPtr @Const Mat mat, @ByPtrPtr @Cast({"uchar**"}) byte[] bArr);

        private native void allocate(@ByPtrPtr @Const Mat mat, @ByPtrPtr @Cast({"uchar**"}) byte[] bArr, int i);

        private native void allocateArray(long j);

        @MemberGetter
        @Cast({"const cv::Mat**"})
        public native PointerPointer arrays();

        @MemberGetter
        @Const
        public native Mat arrays(int i);

        @ByRef
        @Name({"operator ++"})
        public native NAryMatIterator increment();

        @ByVal
        @Name({"operator ++"})
        public native NAryMatIterator increment(int i);

        public native void init(@Cast({"const cv::Mat**"}) PointerPointer pointerPointer, Mat mat, @Cast({"uchar**"}) PointerPointer pointerPointer2, int i);

        public native void init(@ByPtrPtr @Const Mat mat, Mat mat2, @ByPtrPtr @Cast({"uchar**"}) ByteBuffer byteBuffer);

        public native void init(@ByPtrPtr @Const Mat mat, Mat mat2, @ByPtrPtr @Cast({"uchar**"}) ByteBuffer byteBuffer, int i);

        public native void init(@ByPtrPtr @Const Mat mat, Mat mat2, @ByPtrPtr @Cast({"uchar**"}) BytePointer bytePointer);

        public native void init(@ByPtrPtr @Const Mat mat, Mat mat2, @ByPtrPtr @Cast({"uchar**"}) BytePointer bytePointer, int i);

        public native void init(@ByPtrPtr @Const Mat mat, Mat mat2, @ByPtrPtr @Cast({"uchar**"}) byte[] bArr);

        public native void init(@ByPtrPtr @Const Mat mat, Mat mat2, @ByPtrPtr @Cast({"uchar**"}) byte[] bArr, int i);

        public native int narrays();

        public native NAryMatIterator narrays(int i);

        @Cast({"size_t"})
        public native long nplanes();

        public native NAryMatIterator nplanes(long j);

        public native Mat planes();

        public native NAryMatIterator planes(Mat mat);

        @Cast({"uchar*"})
        public native BytePointer ptrs(int i);

        @Cast({"uchar**"})
        public native PointerPointer ptrs();

        public native NAryMatIterator ptrs(int i, BytePointer bytePointer);

        public native NAryMatIterator ptrs(PointerPointer pointerPointer);

        @Cast({"size_t"})
        public native long size();

        public native NAryMatIterator size(long j);

        static {
            Loader.load();
        }

        public NAryMatIterator(Pointer p) {
            super(p);
        }

        public NAryMatIterator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NAryMatIterator position(long position) {
            return (NAryMatIterator) super.position(position);
        }

        public NAryMatIterator() {
            super((Pointer) null);
            allocate();
        }

        public NAryMatIterator(@Cast({"const cv::Mat**"}) PointerPointer arrays, @Cast({"uchar**"}) PointerPointer ptrs, int narrays) {
            super((Pointer) null);
            allocate(arrays, ptrs, narrays);
        }

        public NAryMatIterator(@ByPtrPtr @Const Mat arrays, @ByPtrPtr @Cast({"uchar**"}) BytePointer ptrs) {
            super((Pointer) null);
            allocate(arrays, ptrs);
        }

        public NAryMatIterator(@ByPtrPtr @Const Mat arrays, @ByPtrPtr @Cast({"uchar**"}) BytePointer ptrs, int narrays) {
            super((Pointer) null);
            allocate(arrays, ptrs, narrays);
        }

        public NAryMatIterator(@ByPtrPtr @Const Mat arrays, @ByPtrPtr @Cast({"uchar**"}) ByteBuffer ptrs, int narrays) {
            super((Pointer) null);
            allocate(arrays, ptrs, narrays);
        }

        public NAryMatIterator(@ByPtrPtr @Const Mat arrays, @ByPtrPtr @Cast({"uchar**"}) ByteBuffer ptrs) {
            super((Pointer) null);
            allocate(arrays, ptrs);
        }

        public NAryMatIterator(@ByPtrPtr @Const Mat arrays, @ByPtrPtr @Cast({"uchar**"}) byte[] ptrs, int narrays) {
            super((Pointer) null);
            allocate(arrays, ptrs, narrays);
        }

        public NAryMatIterator(@ByPtrPtr @Const Mat arrays, @ByPtrPtr @Cast({"uchar**"}) byte[] ptrs) {
            super((Pointer) null);
            allocate(arrays, ptrs);
        }

        public NAryMatIterator(@Cast({"const cv::Mat**"}) PointerPointer arrays, Mat planes, int narrays) {
            super((Pointer) null);
            allocate(arrays, planes, narrays);
        }

        public NAryMatIterator(@ByPtrPtr @Const Mat arrays, Mat planes) {
            super((Pointer) null);
            allocate(arrays, planes);
        }

        public NAryMatIterator(@ByPtrPtr @Const Mat arrays, Mat planes, int narrays) {
            super((Pointer) null);
            allocate(arrays, planes, narrays);
        }
    }

    @Namespace("cv")
    public static class MatOp extends Pointer {
        public native void abs(@ByRef @Const MatExpr matExpr, @ByRef MatExpr matExpr2);

        public native void add(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2, @ByRef MatExpr matExpr3);

        public native void add(@ByRef @Const MatExpr matExpr, @ByRef @Const Scalar scalar, @ByRef MatExpr matExpr2);

        public native void assign(@ByRef @Const MatExpr matExpr, @ByRef Mat mat);

        public native void assign(@ByRef @Const MatExpr matExpr, @ByRef Mat mat, int i);

        public native void augAssignAdd(@ByRef @Const MatExpr matExpr, @ByRef Mat mat);

        public native void augAssignAnd(@ByRef @Const MatExpr matExpr, @ByRef Mat mat);

        public native void augAssignDivide(@ByRef @Const MatExpr matExpr, @ByRef Mat mat);

        public native void augAssignMultiply(@ByRef @Const MatExpr matExpr, @ByRef Mat mat);

        public native void augAssignOr(@ByRef @Const MatExpr matExpr, @ByRef Mat mat);

        public native void augAssignSubtract(@ByRef @Const MatExpr matExpr, @ByRef Mat mat);

        public native void augAssignXor(@ByRef @Const MatExpr matExpr, @ByRef Mat mat);

        public native void diag(@ByRef @Const MatExpr matExpr, int i, @ByRef MatExpr matExpr2);

        public native void divide(double d, @ByRef @Const MatExpr matExpr, @ByRef MatExpr matExpr2);

        public native void divide(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2, @ByRef MatExpr matExpr3);

        public native void divide(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2, @ByRef MatExpr matExpr3, double d);

        @Cast({"bool"})
        public native boolean elementWise(@ByRef @Const MatExpr matExpr);

        public native void invert(@ByRef @Const MatExpr matExpr, int i, @ByRef MatExpr matExpr2);

        public native void matmul(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2, @ByRef MatExpr matExpr3);

        public native void multiply(@ByRef @Const MatExpr matExpr, double d, @ByRef MatExpr matExpr2);

        public native void multiply(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2, @ByRef MatExpr matExpr3);

        public native void multiply(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2, @ByRef MatExpr matExpr3, double d);

        public native void roi(@ByRef @Const MatExpr matExpr, @ByRef @Const Range range, @ByRef @Const Range range2, @ByRef MatExpr matExpr2);

        @ByVal
        public native Size size(@ByRef @Const MatExpr matExpr);

        public native void subtract(@ByRef @Const MatExpr matExpr, @ByRef @Const MatExpr matExpr2, @ByRef MatExpr matExpr3);

        public native void subtract(@ByRef @Const Scalar scalar, @ByRef @Const MatExpr matExpr, @ByRef MatExpr matExpr2);

        public native void transpose(@ByRef @Const MatExpr matExpr, @ByRef MatExpr matExpr2);

        public native int type(@ByRef @Const MatExpr matExpr);

        static {
            Loader.load();
        }

        public MatOp(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class MatExpr extends Pointer {
        private native void allocate();

        private native void allocate(@ByRef @Const Mat mat);

        private native void allocate(@Const MatOp matOp, int i);

        private native void allocate(@Const MatOp matOp, int i, @ByRef(nullValue = "cv::Mat()") @Const Mat mat, @ByRef(nullValue = "cv::Mat()") @Const Mat mat2, @ByRef(nullValue = "cv::Mat()") @Const Mat mat3, double d, double d2, @ByRef(nullValue = "cv::Scalar()") @Const Scalar scalar);

        private native void allocateArray(long j);

        @ByRef
        public native Mat a();

        public native MatExpr a(Mat mat);

        public native double alpha();

        public native MatExpr alpha(double d);

        @ByVal
        @Name({"operator ()"})
        public native MatExpr apply(@ByRef @Const Range range, @ByRef @Const Range range2);

        @ByVal
        @Name({"operator ()"})
        public native MatExpr apply(@ByRef @Const Rect rect);

        @ByVal
        @Name({"operator cv::Mat"})
        public native Mat asMat();

        @ByRef
        public native Mat b();

        public native MatExpr b(Mat mat);

        public native double beta();

        public native MatExpr beta(double d);

        @ByRef
        public native Mat c();

        public native MatExpr c(Mat mat);

        @ByVal
        public native MatExpr col(int i);

        @ByVal
        public native Mat cross(@ByRef @Const Mat mat);

        @ByVal
        public native MatExpr diag();

        @ByVal
        public native MatExpr diag(int i);

        public native double dot(@ByRef @Const Mat mat);

        public native int flags();

        public native MatExpr flags(int i);

        @ByVal
        public native MatExpr inv();

        @ByVal
        public native MatExpr inv(int i);

        @ByVal
        public native MatExpr mul(@ByRef @Const Mat mat);

        @ByVal
        public native MatExpr mul(@ByRef @Const Mat mat, double d);

        @ByVal
        public native MatExpr mul(@ByRef @Const MatExpr matExpr);

        @ByVal
        public native MatExpr mul(@ByRef @Const MatExpr matExpr, double d);

        @MemberGetter
        @Const
        public native MatOp op();

        @ByVal
        public native MatExpr row(int i);

        public native MatExpr s(Scalar scalar);

        @ByRef
        public native Scalar s();

        @ByVal
        public native Size size();

        @ByVal
        public native MatExpr t();

        public native int type();

        static {
            Loader.load();
        }

        public MatExpr(Pointer p) {
            super(p);
        }

        public MatExpr(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MatExpr position(long position) {
            return (MatExpr) super.position(position);
        }

        public MatExpr() {
            super((Pointer) null);
            allocate();
        }

        public MatExpr(@ByRef @Const Mat m) {
            super((Pointer) null);
            allocate(m);
        }

        public MatExpr(@Const MatOp _op, int _flags, @ByRef(nullValue = "cv::Mat()") @Const Mat _a, @ByRef(nullValue = "cv::Mat()") @Const Mat _b, @ByRef(nullValue = "cv::Mat()") @Const Mat _c, double _alpha, double _beta, @ByRef(nullValue = "cv::Scalar()") @Const Scalar _s) {
            super((Pointer) null);
            allocate(_op, _flags, _a, _b, _c, _alpha, _beta, _s);
        }

        public MatExpr(@Const MatOp _op, int _flags) {
            super((Pointer) null);
            allocate(_op, _flags);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class FileStorage extends Pointer {
        public static final int APPEND = 2;
        public static final int BASE64 = 64;
        public static final int FORMAT_AUTO = 0;
        public static final int FORMAT_JSON = 24;
        public static final int FORMAT_MASK = 56;
        public static final int FORMAT_XML = 8;
        public static final int FORMAT_YAML = 16;
        public static final int INSIDE_MAP = 4;
        public static final int MEMORY = 4;
        public static final int NAME_EXPECTED = 2;
        public static final int READ = 0;
        public static final int UNDEFINED = 0;
        public static final int VALUE_EXPECTED = 1;
        public static final int WRITE = 1;
        public static final int WRITE_BASE64 = 65;

        private native void allocate();

        private native void allocate(@opencv_core.Str String str, int i);

        private native void allocate(@opencv_core.Str String str, int i, @opencv_core.Str String str2);

        private native void allocate(@opencv_core.Str BytePointer bytePointer, int i);

        private native void allocate(@opencv_core.Str BytePointer bytePointer, int i, @opencv_core.Str BytePointer bytePointer2);

        private native void allocateArray(long j);

        @opencv_core.Str
        public static native String getDefaultObjectName(@opencv_core.Str String str);

        @opencv_core.Str
        public static native BytePointer getDefaultObjectName(@opencv_core.Str BytePointer bytePointer);

        @StdString
        public native BytePointer elname();

        public native FileStorage elname(BytePointer bytePointer);

        public native void endWriteStruct();

        @ByVal
        @Name({"operator []"})
        public native FileNode get(@opencv_core.Str String str);

        @ByVal
        @Name({"operator []"})
        public native FileNode get(@opencv_core.Str BytePointer bytePointer);

        @ByVal
        public native FileNode getFirstTopLevelNode();

        public native int getFormat();

        @ByVal
        @Name({"operator []"})
        public native FileNode getNode(String str);

        @ByVal
        @Name({"operator []"})
        public native FileNode getNode(@Cast({"const char*"}) BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean isOpened();

        @Cast({"bool"})
        public native boolean open(@opencv_core.Str String str, int i);

        @Cast({"bool"})
        public native boolean open(@opencv_core.Str String str, int i, @opencv_core.Str String str2);

        @Cast({"bool"})
        public native boolean open(@opencv_core.Str BytePointer bytePointer, int i);

        @Cast({"bool"})
        public native boolean open(@opencv_core.Str BytePointer bytePointer, int i, @opencv_core.Str BytePointer bytePointer2);

        public native void release();

        @opencv_core.Str
        public native BytePointer releaseAndGetString();

        @ByVal
        public native FileNode root();

        @ByVal
        public native FileNode root(int i);

        public native void startWriteStruct(@opencv_core.Str String str, int i, @opencv_core.Str String str2);

        public native void startWriteStruct(@opencv_core.Str BytePointer bytePointer, int i, @opencv_core.Str BytePointer bytePointer2);

        public native int state();

        public native FileStorage state(int i);

        public native void write(@opencv_core.Str String str, double d);

        public native void write(@opencv_core.Str String str, int i);

        public native void write(@opencv_core.Str String str, @opencv_core.Str String str2);

        public native void write(@opencv_core.Str String str, @ByRef @Const Mat mat);

        public native void write(@opencv_core.Str String str, @ByRef @Const StringVector stringVector);

        public native void write(@opencv_core.Str BytePointer bytePointer, double d);

        public native void write(@opencv_core.Str BytePointer bytePointer, int i);

        public native void write(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        public native void write(@opencv_core.Str BytePointer bytePointer, @ByRef @Const Mat mat);

        public native void write(@opencv_core.Str BytePointer bytePointer, @ByRef @Const StringVector stringVector);

        public native void writeComment(@opencv_core.Str String str);

        public native void writeComment(@opencv_core.Str String str, @Cast({"bool"}) boolean z);

        public native void writeComment(@opencv_core.Str BytePointer bytePointer);

        public native void writeComment(@opencv_core.Str BytePointer bytePointer, @Cast({"bool"}) boolean z);

        public native void writeRaw(@opencv_core.Str String str, @Const Pointer pointer, @Cast({"size_t"}) long j);

        public native void writeRaw(@opencv_core.Str BytePointer bytePointer, @Const Pointer pointer, @Cast({"size_t"}) long j);

        static {
            Loader.load();
        }

        public FileStorage(Pointer p) {
            super(p);
        }

        public FileStorage(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public FileStorage position(long position) {
            return (FileStorage) super.position(position);
        }

        public FileStorage() {
            super((Pointer) null);
            allocate();
        }

        public FileStorage(@opencv_core.Str BytePointer filename, int flags, @opencv_core.Str BytePointer encoding) {
            super((Pointer) null);
            allocate(filename, flags, encoding);
        }

        public FileStorage(@opencv_core.Str BytePointer filename, int flags) {
            super((Pointer) null);
            allocate(filename, flags);
        }

        public FileStorage(@opencv_core.Str String filename, int flags, @opencv_core.Str String encoding) {
            super((Pointer) null);
            allocate(filename, flags, encoding);
        }

        public FileStorage(@opencv_core.Str String filename, int flags) {
            super((Pointer) null);
            allocate(filename, flags);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class FileNode extends Pointer {
        public static final int EMPTY = 16;
        public static final int FLOAT = 2;
        public static final int FLOW = 8;
        public static final int INT = 1;
        public static final int MAP = 5;
        public static final int NAMED = 32;
        public static final int NONE = 0;
        public static final int REAL = 2;
        public static final int SEQ = 4;
        public static final int STR = 3;
        public static final int STRING = 3;
        public static final int TYPE_MASK = 7;
        public static final int UNIFORM = 8;

        private native void allocate();

        private native void allocate(@ByRef @Const FileNode fileNode);

        private native void allocate(@Const FileStorage fileStorage, @Cast({"size_t"}) long j, @Cast({"size_t"}) long j2);

        private native void allocateArray(long j);

        @Cast({"bool"})
        public static native boolean isCollection(int i);

        @Cast({"bool"})
        public static native boolean isEmptyCollection(int i);

        @Cast({"bool"})
        public static native boolean isFlow(int i);

        @Cast({"bool"})
        public static native boolean isMap(int i);

        @Cast({"bool"})
        public static native boolean isSeq(int i);

        @StdString
        @Name({"operator std::string"})
        public native BytePointer asBytePointer();

        @Name({"operator double"})
        public native double asDouble();

        @Name({"operator float"})
        public native float asFloat();

        @Name({"operator int"})
        public native int asInt();

        @ByVal
        @Name({"operator []"})
        public native FileNode at(int i);

        @ByVal
        public native FileNodeIterator begin();

        @Cast({"size_t"})
        public native long blockIdx();

        public native FileNode blockIdx(long j);

        @Cast({"bool"})
        public native boolean empty();

        @ByVal
        public native FileNodeIterator end();

        @MemberGetter
        @Const
        public native FileStorage fs();

        @ByVal
        @Name({"operator []"})
        public native FileNode get(@opencv_core.Str String str);

        @ByVal
        @Name({"operator []"})
        public native FileNode get(@opencv_core.Str BytePointer bytePointer);

        @ByVal
        @Name({"operator []"})
        public native FileNode getNode(String str);

        @ByVal
        @Name({"operator []"})
        public native FileNode getNode(@Cast({"const char*"}) BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean isInt();

        @Cast({"bool"})
        public native boolean isMap();

        @Cast({"bool"})
        public native boolean isNamed();

        @Cast({"bool"})
        public native boolean isNone();

        @Cast({"bool"})
        public native boolean isReal();

        @Cast({"bool"})
        public native boolean isSeq();

        @Cast({"bool"})
        public native boolean isString();

        @ByVal
        public native StringVector keys();

        @ByVal
        public native Mat mat();

        @StdString
        public native BytePointer name();

        @Cast({"size_t"})
        public native long ofs();

        public native FileNode ofs(long j);

        @Cast({"uchar*"})
        public native BytePointer ptr();

        @Cast({"size_t"})
        public native long rawSize();

        public native void readRaw(@opencv_core.Str String str, Pointer pointer, @Cast({"size_t"}) long j);

        public native void readRaw(@opencv_core.Str BytePointer bytePointer, Pointer pointer, @Cast({"size_t"}) long j);

        public native double real();

        public native void setValue(int i, @Const Pointer pointer);

        public native void setValue(int i, @Const Pointer pointer, int i2);

        @Cast({"size_t"})
        public native long size();

        @StdString
        public native BytePointer string();

        public native int type();

        static {
            Loader.load();
        }

        public FileNode(Pointer p) {
            super(p);
        }

        public FileNode(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public FileNode position(long position) {
            return (FileNode) super.position(position);
        }

        public FileNode() {
            super((Pointer) null);
            allocate();
        }

        public FileNode(@Const FileStorage fs, @Cast({"size_t"}) long blockIdx, @Cast({"size_t"}) long ofs) {
            super((Pointer) null);
            allocate(fs, blockIdx, ofs);
        }

        public FileNode(@ByRef @Const FileNode node) {
            super((Pointer) null);
            allocate(node);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class FileNodeIterator extends Pointer {
        private native void allocate();

        private native void allocate(@ByRef @Const FileNode fileNode, @Cast({"bool"}) boolean z);

        private native void allocate(@ByRef @Const FileNodeIterator fileNodeIterator);

        private native void allocateArray(long j);

        @ByRef
        @Name({"operator +="})
        public native FileNodeIterator addPut(int i);

        @Cast({"bool"})
        public native boolean equalTo(@ByRef @Const FileNodeIterator fileNodeIterator);

        @ByRef
        @Name({"operator ++"})
        public native FileNodeIterator increment();

        @ByVal
        @Name({"operator ++"})
        public native FileNodeIterator increment(int i);

        @ByVal
        @Name({"operator *"})
        public native FileNode multiply();

        @ByRef
        public native FileNodeIterator readRaw(@opencv_core.Str String str, Pointer pointer);

        @ByRef
        public native FileNodeIterator readRaw(@opencv_core.Str String str, Pointer pointer, @Cast({"size_t"}) long j);

        @ByRef
        public native FileNodeIterator readRaw(@opencv_core.Str BytePointer bytePointer, Pointer pointer);

        @ByRef
        public native FileNodeIterator readRaw(@opencv_core.Str BytePointer bytePointer, Pointer pointer, @Cast({"size_t"}) long j);

        @Cast({"size_t"})
        public native long remaining();

        static {
            Loader.load();
        }

        public FileNodeIterator(Pointer p) {
            super(p);
        }

        public FileNodeIterator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public FileNodeIterator position(long position) {
            return (FileNodeIterator) super.position(position);
        }

        public FileNodeIterator() {
            super((Pointer) null);
            allocate();
        }

        public FileNodeIterator(@ByRef @Const FileNode node, @Cast({"bool"}) boolean seekEnd) {
            super((Pointer) null);
            allocate(node, seekEnd);
        }

        public FileNodeIterator(@ByRef @Const FileNodeIterator it) {
            super((Pointer) null);
            allocate(it);
        }
    }

    @Namespace("cv::internal")
    @NoOffset
    public static class WriteStructContext extends Pointer {
        private native void allocate(@ByRef FileStorage fileStorage, @opencv_core.Str String str, int i);

        private native void allocate(@ByRef FileStorage fileStorage, @opencv_core.Str String str, int i, @opencv_core.Str String str2);

        private native void allocate(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, int i);

        private native void allocate(@ByRef FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer, int i, @opencv_core.Str BytePointer bytePointer2);

        static {
            Loader.load();
        }

        public WriteStructContext(Pointer p) {
            super(p);
        }

        public WriteStructContext(@ByRef FileStorage _fs, @opencv_core.Str BytePointer name, int flags, @opencv_core.Str BytePointer typeName) {
            super((Pointer) null);
            allocate(_fs, name, flags, typeName);
        }

        public WriteStructContext(@ByRef FileStorage _fs, @opencv_core.Str BytePointer name, int flags) {
            super((Pointer) null);
            allocate(_fs, name, flags);
        }

        public WriteStructContext(@ByRef FileStorage _fs, @opencv_core.Str String name, int flags, @opencv_core.Str String typeName) {
            super((Pointer) null);
            allocate(_fs, name, flags, typeName);
        }

        public WriteStructContext(@ByRef FileStorage _fs, @opencv_core.Str String name, int flags) {
            super((Pointer) null);
            allocate(_fs, name, flags);
        }
    }

    @Namespace("cv")
    public static class MinProblemSolver extends Algorithm {
        @opencv_core.Ptr
        public native Function getFunction();

        @ByVal
        public native TermCriteria getTermCriteria();

        public native double minimize(@ByVal GpuMat gpuMat);

        public native double minimize(@ByVal Mat mat);

        public native double minimize(@ByVal UMat uMat);

        public native void setFunction(@opencv_core.Ptr Function function);

        public native void setTermCriteria(@ByRef @Const TermCriteria termCriteria);

        static {
            Loader.load();
        }

        public MinProblemSolver(Pointer p) {
            super(p);
        }

        public static class Function extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            @Virtual(true)
            @Const({false, false, true})
            public native double calc(@Const DoublePointer doublePointer);

            @Virtual(true)
            @Const({false, false, true})
            public native int getDims();

            @Virtual
            public native void getGradient(@Const DoublePointer doublePointer, DoublePointer doublePointer2);

            @Virtual
            @Const({false, false, true})
            public native double getGradientEps();

            static {
                Loader.load();
            }

            public Function() {
                super((Pointer) null);
                allocate();
            }

            public Function(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public Function(Pointer p) {
                super(p);
            }

            public Function position(long position) {
                return (Function) super.position(position);
            }
        }
    }

    @Namespace("cv")
    public static class DownhillSolver extends MinProblemSolver {
        @opencv_core.Ptr
        public static native DownhillSolver create();

        @opencv_core.Ptr
        public static native DownhillSolver create(@opencv_core.Ptr MinProblemSolver.Function function, @ByVal(nullValue = "cv::InputArray(cv::Mat_<double>(1,1,0.0))") GpuMat gpuMat, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,5000,0.000001)") TermCriteria termCriteria);

        @opencv_core.Ptr
        public static native DownhillSolver create(@opencv_core.Ptr MinProblemSolver.Function function, @ByVal(nullValue = "cv::InputArray(cv::Mat_<double>(1,1,0.0))") Mat mat, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,5000,0.000001)") TermCriteria termCriteria);

        @opencv_core.Ptr
        public static native DownhillSolver create(@opencv_core.Ptr MinProblemSolver.Function function, @ByVal(nullValue = "cv::InputArray(cv::Mat_<double>(1,1,0.0))") UMat uMat, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,5000,0.000001)") TermCriteria termCriteria);

        public native void getInitStep(@ByVal GpuMat gpuMat);

        public native void getInitStep(@ByVal Mat mat);

        public native void getInitStep(@ByVal UMat uMat);

        public native void setInitStep(@ByVal GpuMat gpuMat);

        public native void setInitStep(@ByVal Mat mat);

        public native void setInitStep(@ByVal UMat uMat);

        static {
            Loader.load();
        }

        public DownhillSolver(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class ConjGradSolver extends MinProblemSolver {
        @opencv_core.Ptr
        public static native ConjGradSolver create();

        @opencv_core.Ptr
        public static native ConjGradSolver create(@opencv_core.Ptr MinProblemSolver.Function function, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS,5000,0.000001)") TermCriteria termCriteria);

        static {
            Loader.load();
        }

        public ConjGradSolver(Pointer p) {
            super(p);
        }
    }
}
