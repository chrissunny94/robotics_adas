

# This is a basic CUDA walk through 



- basic_example , of add two numbers
   
   - CPU based example compiled with G++
   - GPU based example compiled with nvcc
        - turn our add function into a function that the GPU can run, called a kernel in CUDA. To do this, all I have to do is add the specifier __global__
        - __global__ function is known as a CUDA kernel, and runs on the GPU. Code that runs on the GPU is often called device code, while code that runs on the CPU is host code .
        - To allocate data in unified memory, call cudaMallocManaged(), which returns a pointer that you can access from host (CPU) code or device (GPU) code. To free the data, just pass the pointer to cudaFree().
        






# References

- https://developer.nvidia.com/blog/even-easier-introduction-cuda/