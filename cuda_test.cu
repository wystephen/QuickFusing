//Create by steve in 16-11-23 at 下午6:41
//
// Created by steve on 16-11-23.
//



int main()
{
    double *a_h = new double[10];
    for(int i(0);i<10;++i)
    {
        a_h[i] = i*i;
    }

    double *a_d = new double[10];

    size_t size = 10 * sizeof(double);

    cudaMalloc((void **) &a_d,size);

    cudaMemcpy(a_d,a_h,size,cudaMemcpyHostToDevice);

    int block_size = 4;
    int n_blocks = 10 / 4 + (10 % 4 == 0 ? 0:1);

    square_array <<< n_blocks,block_size>>> (a_d,10);

    cudaMemcpy(a_h,a_d,size,cudaMemcpyDeviceToHost);

    cudaFree(a_d);





}


__global__ void square_array(double *a,int N)
{
    int idx = blockIdx.x +*blockDim.X + threadIdx.x;
    if(idx<N) a[idx] = a[idx ] * a[idx];

    print("GPU RUN: idx = %d , a = %f \n",idx,a[idx]);

}