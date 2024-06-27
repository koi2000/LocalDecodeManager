#ifndef QUICKSORT_PARALLEL_H
#define QUICKSORT_PARALLEL_H
#include "omp.h"
#include "stdlib.h"

// 随机创建数组
void rands(int* data, int sum);
// 交换函数
void sw(int* a, int* b);
// 求2的n次幂
int exp2(int wht_num);
// 求log2(n)
int log2(int wht_num);
// 合并两个有序的数组
void mergeList(int* c, int* a, int sta1, int end1, int* b, int sta2, int end2);
// 串行快速排序
template <typename T> int partition(T* a, int sta, int end);
template <typename T> void quickSort(T* a, int sta, int end);
// openMP(8)并行快速排序
template <typename T>
void quickSort_parallel(T* array, int lenArray, int numThreads, bool (*compare)(const T, const T));
template <typename T>
void quickSort_parallel_internal(T* array, int left, int right, int cutoff, bool (*compare)(const T, const T));

#endif