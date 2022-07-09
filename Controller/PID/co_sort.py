# Author: Ajay Tak
# Date: 30-06-2022

def co_sorting(arr):
    l = len(arr)
    arr1 =[]
    arr2 =[]
    arr3 =[]
    for i in range(l):
        arr1.append(arr[i][0])
        arr2.append(arr[i][1])
        arr3.append(arr[i][2])
    return [arr1, arr2, arr3]
    