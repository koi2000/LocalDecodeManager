#include "quickSort.h"

void rands(int* data, int sum) {
    int i;
    for (i = 0; i < sum; i++) {
        data[i] = rand() % 100000000;
    }
}

void sw(int* a, int* b) {
    int t = *a;
    *a = *b;
    *b = t;
}
int exp2(int wht_num) {
    int wht_i;
    wht_i = 1;
    while (wht_num > 0) {
        wht_num--;
        wht_i = wht_i * 2;
    }
    return wht_i;
}
int log2(int wht_num) {
    int wht_i, wht_j;
    wht_i = 1;
    wht_j = 2;
    while (wht_j < wht_num) {
        wht_j = wht_j * 2;
        wht_i++;
    }
    if (wht_j > wht_num)
        wht_i--;
    return wht_i;
}

template <typename T> int partition(int* a, int sta, int end) {
    int i = sta, j = end + 1;
    int x = a[sta];
    while (1) {
        while (a[++i] < x && i < end)
            ;
        while (a[--j] > x)
            ;
        if (i >= j)
            break;
        sw(&a[i], &a[j]);
    }
    a[sta] = a[j];
    a[j] = x;
    return j;
}

template <typename T>
void quickSort_parallel_internal(T* a, int left, int right, int cutoff, bool (*compare)(const T, const T)) {
    int i = left, j = right;
    T tmp;
    T pivot = a[(left + right) / 2];
    // 进行数组分割，分成两部分（符合左小右大）
    while (i <= j) {
        while (compare(a[i], pivot))
            i++;
        while (!compare(a[j], pivot))
            j--;
        if (i <= j) {
            tmp = a[i];
            a[i] = a[j];
            a[j] = tmp;
            i++;
            j--;
        }
    }
    // int j = partition(a,left,right);
    if (((right - left) < cutoff)) {
        if (left < j) {
            quickSort_parallel_internal(a, left, j, cutoff, compare);
        }
        if (i < right) {
            quickSort_parallel_internal(a, i, right, cutoff, compare);
        }
    } else {
#pragma omp task
        /*  task是“动态”定义任务的，在运行过程中，
            只需要使用task就会定义一个任务，
            任务就会在一个线程上去执行，那么其它的任务就可以并行的执行。
            可能某一个任务执行了一半的时候，或者甚至要执行完的时候，
            程序可以去创建第二个任务，任务在一个线程上去执行，一个动态的过程
        */
        // 对两部分再进行并行的线程排序
        { quickSort_parallel_internal(a, left, j, cutoff, compare); }
#pragma omp task
        { quickSort_parallel_internal(a, i, right, cutoff, compare); }
    }
}

// 并行openMP排序
template <typename T> void quickSort_parallel(T* a, int lenArray, int numThreads, bool (*compare)(const T, const T)) {
    int cutoff = 1000;
#pragma omp parallel num_threads(numThreads)  // 指定线程数的数量
    {
#pragma omp single  // 串行执行
        { quickSort_parallel_internal(a, 0, lenArray - 1, cutoff, compare); }
    }
}

// 合并两个已排序的数组
template <typename T> void mergeList(int* c, int* a, int sta1, int end1, int* b, int sta2, int end2) {
    int a_index = sta1;  // 遍历数组a的下标
    int b_index = sta2;  // 遍历数组b的下标
    int i = 0;           // 记录当前存储位置
    // int *c;
    // c = (int *)malloc(sizeof(int) * (end1 - sta1 + 1 + end2 - sta2 + 1));
    while (a_index < end1 && b_index < end2) {
        if (a[a_index] <= b[b_index]) {
            c[i] = a[a_index];
            a_index++;
        } else {
            c[i] = b[b_index];
            b_index++;
        }
        i++;
    }
    while (a_index < end1) {
        c[i] = a[a_index];
        i++;
        a_index++;
    }
    while (b_index < end2) {
        c[i] = b[b_index];
        i++;
        b_index++;
    }
}
// 串行快速排序
template <typename T> void quickSort(int* a, int sta, int end) {
    if (sta < end) {
        // printf("3\n");
        int mid = partition(a, sta, end);
        quickSort(a, sta, mid - 1);
        quickSort(a, mid + 1, end);
    }
}