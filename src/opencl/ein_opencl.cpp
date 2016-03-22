
#include "ein_words.h"
#include "ein.h"
#include <boost/filesystem.hpp>

#include <oclUtils.h>
#include <shrQATest.h>

#ifdef USE_OPENCL
using namespace boost::filesystem;
int runFunction(int argc, char** argv) ;

cl_platform_id cpPlatform;          // OpenCL Platform
 
struct GaussianMapGPU {
  cl_mem cmPinnedBufIn;               // OpenCL host memory input buffer object:  pinned 
  cl_mem cmPinnedBufOut;              // OpenCL host memory output buffer object:  pinned
  cl_mem* cmDevBufIn;                 // OpenCL device memory input buffer object  
  cl_mem* cmDevBufOut;                // OpenCL device memory output buffer object
  cl_float* uiInput = NULL;            // Mapped Pointer to pinned Host input buffer for host processing
  cl_float* uiOutput = NULL;           // Mapped Pointer to pinned Host output buffer for host processing
};

struct ImageGPU {
  cl_mem cmPinnedBufIn;               // OpenCL host memory input buffer object:  pinned 
  cl_mem* cmDevBufIn;                 // OpenCL device memory input buffer object  
  cl_float* uiInput = NULL;            // Mapped Pointer to pinned Host input buffer for host processing
};

void gaussianMapEcho(shared_ptr<GaussianMap> in, shared_ptr<GaussianMap> out) {

/*
  GaussianMapGPU t_gmgpu;

  cl_int ciErrNum;			        // Error code var


  int szBuffBytes = in->width * in->height * sizeof(float);

  cl_command_queue* t_cqCommandQueue;   // OpenCL command queue array
  t_cqCommandQueue = new cl_command_queue[GpuDevMngr->uiUsefulDevCt];
  t_cqCommandQueue[i] = clCreateCommandQueue(cxGPUContext, GpuDevMngr->cdDevices[GpuDevMngr->uiUsefulDevs[i]], 0, &ciErrNum);

  //cmPinnedBufOut = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, szBuffBytes, NULL, &ciErrNum);
  //uiOutput = (cl_uint*)clEnqueueMapBuffer(t_cqCommandQueue[0], cmPinnedBufOut, CL_TRUE, CL_MAP_READ, 0, szBuffBytes, 0, NULL, NULL, &ciErrNum);
  //cmDevBufOut = new cl_mem[GpuDevMngr->uiUsefulDevCt];

  t_gmgpu.cmPinnedBufIn = clCreateBuffer(cxGPUContext, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, szBuffBytes, NULL, &ciErrNum);
  t_gmgpu.uiInput = (cl_uint*)clEnqueueMapBuffer(t_cqCommandQueue[0], t_gmgpu.PinnedBufIn, CL_TRUE, CL_MAP_WRITE | CL_MAP_READ, 0, szBuffBytes, 0, NULL, NULL, &ciErrNum);
  //uiInput = (cl_uint*)clEnqueueMapBuffer(t_cqCommandQueue[0], cmPinnedBufIn, CL_TRUE, CL_MAP_WRITE, 0, szBuffBytes, 0, NULL, NULL, &ciErrNum);

  t_gmgpu.cmDevBufIn = new cl_mem[1];









*/

  //Get the NVIDIA platform
  cl_int ciErrNum = oclGetPlatformID(&cpPlatform);
  printf("platform: %d\n", ciErrNum);
  oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
  shrLog("clGetPlatformID...\n\n"); 
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
  shrLog("Get the Device info and select Device...\n");
  ciErrNum = clGetDeviceIDs(cpPlatform, CL_DEVICE_TYPE_GPU, 0, NULL, &uiNumDevices);
  oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
  cdDevices = (cl_device_id *)malloc(uiNumDevices * sizeof(cl_device_id) );
  ciErrNum = clGetDeviceIDs(cpPlatform, CL_DEVICE_TYPE_GPU, uiNumDevices, cdDevices, NULL);
  oclCheckErrorEX(ciErrNum, CL_SUCCESS, pCleanup);
*/


}

namespace ein_words {

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
