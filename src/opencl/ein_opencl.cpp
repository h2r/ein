
#include "ein_words.h"
#include "ein.h"
#include <boost/filesystem.hpp>

#ifdef USE_OPENCL

#include <oclUtils.h>
#include <shrQATest.h>

using namespace boost::filesystem;
int runFunction(int argc, char** argv) ;

// Helper to clean up
//*****************************************************************************
void Cleanup(int iExitCode)
{
/*
//    shrLog("\nStarting Cleanup...\n\n");

    // Cleanup allocated objects
    if(nbodyGPU)delete nbodyGPU;
    if(cqCommandQueue)clReleaseCommandQueue(cqCommandQueue);
    if(cxContext)clReleaseContext(cxContext);
    if(hPos)delete [] hPos;
    if(hVel)delete [] hVel;
    if(hColor)delete [] hColor;
    if(renderer)delete renderer;

    // finalize logs and leave
    if (bNoPrompt || bQATest)
    {
//        shrLogEx(LOGBOTH | CLOSELOG, 0, "%s Exiting...\n", cExecutablePath);
    }
    else 
    {
        shrLogEx(LOGBOTH | CLOSELOG, 0, "%s Exiting...\nPress <Enter> to Quit\n", cExecutablePath);
        #ifdef WIN32
            getchar();
        #endif
    }
*/
    ROS_ERROR_STREAM("Oops, hit Cleanup in OpenCL code...");
    exit (iExitCode);
}
void (*pCleanup)(int) = &Cleanup;

typedef struct GaussianMapGpu GaussianMapGpu;

struct EinGpuConfig {
  EinGpuConfig() {
    //Get the NVIDIA platform
    cl_int ciErrNum = oclGetPlatformID(&cpPlatform);
    printf("platform: %d\n", ciErrNum);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
    shrLog("clGetPlatformID...\n\n"); 

    shrLog("Get the Device info and select Device...\n");
    ciErrNum = clGetDeviceIDs(cpPlatform, CL_DEVICE_TYPE_GPU, 0, NULL, &uiNumDevices);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
    cdDevices = (cl_device_id *)malloc(uiNumDevices * sizeof(cl_device_id) );
    ciErrNum = clGetDeviceIDs(cpPlatform, CL_DEVICE_TYPE_GPU, uiNumDevices, cdDevices, NULL);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);

    // Set target device and Query number of compute units on uiTargetDevice
    shrLog("  # of Devices Available = %u\n", uiNumDevices); 
    uiTargetDevice = CLAMP(uiTargetDevice, 0, (uiNumDevices - 1));
    shrLog("  Using Device %u, ", uiTargetDevice); 
    oclPrintDevName(LOGBOTH, cdDevices[uiTargetDevice]);  
    cl_uint uiNumComputeUnits;        
    clGetDeviceInfo(cdDevices[uiTargetDevice], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(uiNumComputeUnits), &uiNumComputeUnits, NULL);
    shrLog("  # of Compute Units = %u\n", uiNumComputeUnits); 

    //Create the CPU context
    shrLog("clCreateContext...\n"); 
    cxContext = clCreateContext(0, uiNumDevsUsed, &cdDevices[uiTargetDevice], NULL, NULL, &ciErrNum);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);

    // Create a command-queue 
    shrLog("clCreateCommandQueue...\n\n"); 
    cqCommandQueue = clCreateCommandQueue(cxContext, cdDevices[uiTargetDevice], CL_QUEUE_PROFILING_ENABLE, &ciErrNum);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
  }

  ~EinGpuConfig() {
    if(cdDevices)free(cdDevices);
    if(cxContext)clReleaseContext(cxContext);
    if(cSourceCL)free(cSourceCL);             // Buffer to hold source for compilation 

    if(cpProgram)clReleaseProgram(cpProgram);
    if(ckSobel)clReleaseKernel(ckSobel);
  }

  cl_platform_id cpPlatform;          // OpenCL Platform
  cl_context cxContext;               // OpenCL Context
  cl_command_queue cqCommandQueue;    // OpenCL Command Queue

  cl_device_id *cdDevices = NULL;     // OpenCL device list
  cl_uint uiNumDevices = 0;           // Number of OpenCL devices available
  cl_uint uiNumDevsUsed = 1;          // Number of OpenCL devices used in this sample 
  cl_uint uiTargetDevice = 0;	      // OpenCL Device to compute on

  size_t szKernelLength;	      // Byte size of kernel code
  char* cSourceCL = NULL;             // Buffer to hold source for compilation 
  cl_program cpProgram;               // OpenCL program
  cl_kernel ckSobel;                 // OpenCL Kernel array for Sobel

  const char* clSourcefile = "SobelFilter.cl";  // OpenCL kernel source file
  char* cPathAndName = NULL;          // var for full paths to data, src, etc.

  void programBuild();
  void kernelCreate(shared_ptr<GaussianMapGpu> gmgpu);
  void kernelRun(shared_ptr<GaussianMapGpu> gmgpu);
  void kernelReturn(shared_ptr<GaussianMapGpu> gmgpu);
};
 
struct GaussianMapGpu {

  GaussianMapGpu() { cpuReflection = NULL; }

  GaussianMapGpu(shared_ptr<GaussianMap> _cpuReflection, shared_ptr<EinGpuConfig> _gConfig) { 
    cpuReflection = _cpuReflection; 
    gConfig = _gConfig;
    szBuffBytes = cpuReflection->width * cpuReflection->height * sizeof(float);

    // Allocate pinned input and output host image buffers:  mem copy operations to/from pinned memory is much faster than paged memory
    cl_int ciErrNum = CL_SUCCESS;
    cmPinnedBufIn = clCreateBuffer(gConfig->cxContext, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, szBuffBytes, NULL, &ciErrNum);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
    cmPinnedBufOut = clCreateBuffer(gConfig->cxContext, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, szBuffBytes, NULL, &ciErrNum);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
    shrLog("\nclCreateBuffer (Input and Output Pinned Host buffers)...\n"); 

    // Get mapped pointers for writing to pinned input and output host image pointers 
    uiInput = (cl_float*)clEnqueueMapBuffer(gConfig->cqCommandQueue, cmPinnedBufIn, CL_TRUE, CL_MAP_WRITE, 0, szBuffBytes, 0, NULL, NULL, &ciErrNum);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
    uiOutput = (cl_float*)clEnqueueMapBuffer(gConfig->cqCommandQueue, cmPinnedBufOut, CL_TRUE, CL_MAP_READ, 0, szBuffBytes, 0, NULL, NULL, &ciErrNum);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
    shrLog("clEnqueueMapBuffer (Pointer to Input and Output pinned host buffers)...\n"); 

    // Create the device buffers in GMEM on each device
    cmDevBufIn = clCreateBuffer(gConfig->cxContext, CL_MEM_READ_ONLY, szBuffBytes, NULL, &ciErrNum);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
    cmDevBufOut = clCreateBuffer(gConfig->cxContext, CL_MEM_WRITE_ONLY, szBuffBytes, NULL, &ciErrNum);
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
    shrLog("clCreateBuffer (Input and Output GMEM buffers)...\n"); 
  }

  ~GaussianMapGpu() { 
    if(uiInput)clEnqueueUnmapMemObject(gConfig->cqCommandQueue, cmPinnedBufIn, (void*)uiInput, 0, NULL, NULL);
    if(uiOutput)clEnqueueUnmapMemObject(gConfig->cqCommandQueue, cmPinnedBufOut, (void*)uiOutput, 0, NULL, NULL);
    if(cmPinnedBufIn)clReleaseMemObject(cmPinnedBufIn);
    if(cmPinnedBufOut)clReleaseMemObject(cmPinnedBufOut);
  }

  // XXX consider 
  // a: using unpinned memory pointing to the reflection's memory
  // b: using pinned memory and replacing the reflection's memory with the new pinned memory
  // for now: duplicate memory since it is float here and double there (~errr)
  shared_ptr<GaussianMap> cpuReflection;
  shared_ptr<EinGpuConfig> gConfig;
  size_t szBuffBytes;                 // Size of main image buffers
  cl_mem cmPinnedBufIn;               // OpenCL host memory input buffer object:  pinned 
  cl_mem cmPinnedBufOut;              // OpenCL host memory output buffer object:  pinned
  cl_mem cmDevBufIn;                 // OpenCL device memory input buffer object  
  cl_mem cmDevBufOut;                // OpenCL device memory output buffer object
  cl_float* uiInput = NULL;            // Mapped Pointer to pinned Host input buffer for host processing
  cl_float* uiOutput = NULL;           // Mapped Pointer to pinned Host output buffer for host processing
};

struct ImageGpu {
  cl_mem cmPinnedBufIn;               // OpenCL host memory input buffer object:  pinned 
  cl_mem* cmDevBufIn;                 // OpenCL device memory input buffer object  
  cl_float* uiInput = NULL;            // Mapped Pointer to pinned Host input buffer for host processing
};


void gaussianMapGpuEchoTest(shared_ptr<GaussianMap> gm_in) {

  shared_ptr<EinGpuConfig> gc = make_shared<EinGpuConfig>(); 
  cout << "Made config." << endl;
  shared_ptr<GaussianMapGpu> gmgpu_out = make_shared<GaussianMapGpu>(gm_in, gc);

/*

double SobelFilterGPU(cl_uint* uiInputImage, cl_uint* uiOutputImage)
{
    // If this is a video application, fresh data in pinned host buffer is needed beyond here 
    //      This line could be a sync point assuring that an asynchronous acqusition is complete.
    //      That ascynchronous acquisition would do a map, update and unmap for the pinned input buffer
    //
    //      Otherwise a synchronous acquisition call ('get next frame') could be placed here, but that would be less optimal.

    // For each device: copy fresh input H2D 
    ciErrNum = CL_SUCCESS;
    for (cl_uint i = 0; i < GpuDevMngr->uiUsefulDevCt; i++)
    {
        // Nonblocking Write of input image data from host to device
        ciErrNum |= clEnqueueWriteBuffer(cqCommandQueue[i], cmDevBufIn[i], CL_FALSE, 0, szAllocDevBytes[i], 
                                        (void*)&uiInputImage[uiInHostPixOffsets[i]], 0, NULL, NULL);
    }

    // Sync all queues to host and start computation timer on host to get computation elapsed wall clock  time
    // (Only for timing... can be omitted in a production app)
    for (cl_uint j = 0; j < GpuDevMngr->uiUsefulDevCt; j++)
    {
        ciErrNum |= clFinish(cqCommandQueue[j]);
    }
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);

    // For each device: Process
    shrDeltaT(0);
    for (cl_uint i = 0; i < GpuDevMngr->uiUsefulDevCt; i++)
    {
        // Determine configuration bytes, offsets and launch config, based on position of device region vertically in image
        if (GpuDevMngr->uiUsefulDevCt == 1)
        {
            // One device processes the whole image with no offset tricks needed
            szGlobalWorkSize[1] = shrRoundUp((int)szLocalWorkSize[1], (int)uiDevImageHeight[i]);
        }
        else if (i == 0)
        {
            // Multiple devices, top boundary tile:  
            // Process whole device allocation, including extra row 
            // No offset, but don't return the last row (dark/garbage row) D2H 
            szGlobalWorkSize[1] = shrRoundUp((int)szLocalWorkSize[1], (int)uiDevImageHeight[i]);
        }
        else if (i < (GpuDevMngr->uiUsefulDevCt - 1))
        {
            // Multiple devices, middle tile:  
            // Process whole device allocation, including extra 2 rows 
            // Offset down by 1 row, and don't return the first and last rows (dark/garbage rows) D2H 
            szGlobalWorkSize[1] = shrRoundUp((int)szLocalWorkSize[1], (int)uiDevImageHeight[i]);
        }
        else 
        {   
            // Multiple devices, last boundary tile:  
            // Process whole device allocation, including extra row 
            // Offset down by 1 row, and don't return the first row (dark/garbage row) D2H 
            szGlobalWorkSize[1] = shrRoundUp((int)szLocalWorkSize[1], (int)uiDevImageHeight[i]);
        }

        // Pass in dev image height (# of rows worked on) for this device
        ciErrNum |= clSetKernelArg(ckSobel[i], 5, sizeof(cl_uint), (void*)&uiDevImageHeight[i]);

        // Launch Sobel kernel(s) into queue(s) and push to device(s)
        ciErrNum |= clEnqueueNDRangeKernel(cqCommandQueue[i], ckSobel[i], 2, NULL, szGlobalWorkSize, szLocalWorkSize, 0, NULL, NULL);

        // Push to device(s) so subsequent clFinish in queue 0 doesn't block driver from issuing enqueue command for higher queues
        ciErrNum |= clFlush(cqCommandQueue[i]);
    }

    // Sync all queues to host and get elapsed wall clock time for computation in all queues
    // (Only for timing... can be omitted in a production app)
    for (cl_uint j = 0; j < GpuDevMngr->uiUsefulDevCt; j++)
    {
        ciErrNum |= clFinish(cqCommandQueue[j]);
    }
    double dKernelTime = shrDeltaT(0); // Time from launch of first compute kernel to end of all compute kernels 
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);

    // For each device: copy fresh output D2H
    for (cl_uint i = 0; i < GpuDevMngr->uiUsefulDevCt; i++)
    {
        // Determine configuration bytes and offsets based on position of device region vertically in image
        size_t szReturnBytes;
        cl_uint uiOutDevByteOffset;        
        if (GpuDevMngr->uiUsefulDevCt == 1)
        {
            // One device processes the whole image with no offset tricks needed
            szReturnBytes = szBuffBytes;
            uiOutDevByteOffset = 0;
        } 
        else if (i == 0)
        {
            // Multiple devices, top boundary tile:  
            // Process whole device allocation, including extra row 
            // No offset, but don't return the last row (dark/garbage row) D2H 
            szReturnBytes = szAllocDevBytes[i] - (uiImageWidth * sizeof(cl_uint));
            uiOutDevByteOffset = 0;
        }
        else if (i < (GpuDevMngr->uiUsefulDevCt - 1))
        {
            // Multiple devices, middle tile:  
            // Process whole device allocation, including extra 2 rows 
            // Offset down by 1 row, and don't return the first and last rows (dark/garbage rows) D2H 
            szReturnBytes = szAllocDevBytes[i] - ((uiImageWidth * sizeof(cl_uint)) * 2);
            uiOutDevByteOffset = uiImageWidth * sizeof(cl_uint);
        }        
        else 
        {   
            // Multiple devices, last boundary tile:  
            // Process whole device allocation, including extra row 
            // Offset down by 1 row, and don't return the first row (dark/garbage row) D2H 
            szReturnBytes = szAllocDevBytes[i] - (uiImageWidth * sizeof(cl_uint));
            uiOutDevByteOffset = uiImageWidth * sizeof(cl_uint);
        }        
        
        // Non Blocking Read of output image data from device to host 
        ciErrNum |= clEnqueueReadBuffer(cqCommandQueue[i], cmDevBufOut[i], CL_FALSE, uiOutDevByteOffset, szReturnBytes, 
                                       (void*)&uiOutputImage[uiOutHostPixOffsets[i]], 0, NULL, NULL);
    }

    // Finish all queues and check for errors before returning 
    // The block here assures valid output data for subsequent host processing
    for (cl_uint j = 0; j < GpuDevMngr->uiUsefulDevCt; j++)
    {
        ciErrNum |= clFinish(cqCommandQueue[j]);
    }
    oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);

    return dKernelTime;
}

*/

/*
      
      if (bDouble)
      {
	      shrLog("Double precision execution...\n\n");
      }
      else
      {
	      shrLog("Single precision execution...\n\n");
      }

      flopsPerInteraction = bDouble ? 30 : 20; 
  
      //Get all the devices
*/
}

void EinGpuConfig::programBuild() {
  // Read the kernel in from file
  free(cPathAndName);
  oclCheckErrorEX(cPathAndName != NULL, shrTRUE, pCleanup);
  cSourceCL = oclLoadProgSource(cPathAndName, "// My comment\n", &szKernelLength);
  oclCheckErrorEX(cSourceCL != NULL, shrTRUE, pCleanup);
  shrLog("Load OpenCL Prog Source from File...\n"); 

  // Create the program object
  cl_int ciErrNum = oclGetPlatformID(&cpPlatform);
  cpProgram = clCreateProgramWithSource(cxContext, 1, (const char **)&cSourceCL, &szKernelLength, &ciErrNum);
  oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
  shrLog("clCreateProgramWithSource...\n"); 

  // Build the program with 'mad' Optimization option
//#ifdef MAC
  //char *flags = "-cl-fast-relaxed-math -DMAC";
//#else
  //char *flags = "-cl-fast-relaxed-math";
//#endif
  const char *flags = "";

  ciErrNum = clBuildProgram(cpProgram, 0, NULL, flags, NULL, NULL);
  if (ciErrNum != CL_SUCCESS)
  {
      // On error: write out standard error, Build Log and PTX, then cleanup and exit
      shrLogEx(LOGBOTH | ERRORMSG, ciErrNum, STDERROR);
      oclLogBuildInfo(cpProgram, oclGetFirstDev(cxContext));
      oclLogPtx(cpProgram, oclGetFirstDev(cxContext), "oclSobelFilter.ptx");
      //shrQAFinish(argc, (const char **)argv, QA_FAILED);
      Cleanup(EXIT_FAILURE);
  }
  shrLog("clBuildProgram...\n\n"); 
}

void EinGpuConfig::kernelCreate(shared_ptr<GaussianMapGpu> gmgpu) {
  cl_int ciErrNum = CL_SUCCESS;
  // Create kernel instance
  ckSobel = clCreateKernel(cpProgram, "ckSobel", &ciErrNum);
  oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
  shrLog("clCreateKernel (ckSobel)...\n"); 

  // Set the common argument values for the Median kernel instance for each device
  int iBlockDimX = 1;
  int iBlockDimY = 2;
  int iLocalPixPitch = iBlockDimX + 2;
  int uiImageWidth = 10;
  float fThresh = 1.0;
  ciErrNum = clSetKernelArg(ckSobel, 0, sizeof(cl_mem), (void*)&(gmgpu->cmDevBufIn));
  ciErrNum |= clSetKernelArg(ckSobel, 1, sizeof(cl_mem), (void*)&(gmgpu->cmDevBufOut));
  ciErrNum |= clSetKernelArg(ckSobel, 2, (iLocalPixPitch * (iBlockDimY + 2) * sizeof(cl_uchar4)), NULL);
  ciErrNum |= clSetKernelArg(ckSobel, 3, sizeof(cl_int), (void*)&iLocalPixPitch);
  ciErrNum |= clSetKernelArg(ckSobel, 4, sizeof(cl_uint), (void*)&uiImageWidth);
  ciErrNum |= clSetKernelArg(ckSobel, 6, sizeof(cl_float), (void*)&fThresh);        
  oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
  shrLog("clSetKernelArg (0-4), Device %u...\n"); 
}

void EinGpuConfig::kernelRun(shared_ptr<GaussianMapGpu> gmgpu) {
  cl_int ciErrNum = CL_SUCCESS;
  // For each device: copy fresh input H2D 

  int input_offset = 10;
  // Nonblocking Write of input image data from host to device
  ciErrNum |= clEnqueueWriteBuffer(cqCommandQueue, gmgpu->cmDevBufIn, CL_FALSE, 0, gmgpu->szBuffBytes, 
				  (void*)&(gmgpu->uiInput)[input_offset], 0, NULL, NULL);

  // Sync all queues to host and start computation timer on host to get computation elapsed wall clock  time
  // (Only for timing... can be omitted in a production app)
  ciErrNum |= clFinish(cqCommandQueue);
  oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);

  // Pass in dev image height (# of rows worked on) for this device
  int heightArg = 10;
  ciErrNum |= clSetKernelArg(ckSobel, 5, sizeof(cl_float), (void*)&heightArg);

  // Launch Sobel kernel(s) into queue(s) and push to device(s)
  size_t szLocalWorkSize[2] = {10, 10};
  size_t szGlobalWorkSize[2] = {10, 10};
  ciErrNum |= clEnqueueNDRangeKernel(cqCommandQueue, ckSobel, 2, NULL, szGlobalWorkSize, szLocalWorkSize, 0, NULL, NULL);

  // Push to device(s) so subsequent clFinish in queue 0 doesn't block driver from issuing enqueue command for higher queues
  ciErrNum |= clFlush(cqCommandQueue);

  // Sync all queues to host and get elapsed wall clock time for computation in all queues
  // (Only for timing... can be omitted in a production app)
  ciErrNum |= clFinish(cqCommandQueue);
}

void EinGpuConfig::kernelReturn(shared_ptr<GaussianMapGpu> gmgpu) {
  cl_int ciErrNum = CL_SUCCESS;

  int output_offset = 10;
  // Non Blocking Read of output image data from device to host 
  ciErrNum |= clEnqueueReadBuffer(cqCommandQueue, gmgpu->cmDevBufOut, CL_FALSE, 0, gmgpu->szBuffBytes, 
				 (void*)&(gmgpu->uiOutput)[output_offset], 0, NULL, NULL);

  // Finish all queues and check for errors before returning 
  // The block here assures valid output data for subsequent host processing
  ciErrNum |= clFinish(cqCommandQueue);
  oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
}


namespace ein_words {

WORD(OpenClEchoTest)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<GaussianMap> a_map = make_shared<GaussianMap>(100,100,0.1);
  gaussianMapGpuEchoTest( a_map );
}
END_WORD
REGISTER_WORD(OpenClEchoTest)

WORD(OpenClNbodyDemo)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string programName = "einOpenClNbodyDemo";
  const char *cstr = programName.c_str();
  char ** arg = (char **) & cstr;
  runFunction(1, arg);
}
END_WORD
REGISTER_WORD(OpenClNbodyDemo)









}
#endif
