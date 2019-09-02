#include <vector>
#include <iostream>
#include <stack>
using namespace std;

class Solution {
public:
    int maximalRectangle(vector<vector<char>>& matrix) {
        if(matrix.size() <= 0 || matrix[0].size() <= 0){
            return 0;
        }
        int max_num = 0;
        vector<int> heigh(matrix[0].size());
        for(int i =0;i < matrix.size();i++){
            for(int j =0; j < matrix[0].size();j++){
                if(i == 0){
                    heigh[j] = matrix[i][j] ? 1 : 0;
                }else{
                    heigh[j] = matrix[i][j] ? matrix[i-1][j] + 1 : 0; 
                }
            }
            max_num = max(max_num,this->findRaw(heigh));
        }
        return max_num;
    }
    int findRaw(vector<int>& heigh){
        if(heigh.size() <=0){
            return 0;
        }
        stack<int> stack;
        stack.push(heigh[0]);
        int max_num = heigh[0];
        int i = 1;
        while(i < heigh.size() || !stack.empty()){
            if(i < heigh.size() && (stack.empty() || heigh[i-1] < heigh[i])){
                stack.push(i);
                i++;
            }else{
                int tmp = stack.top();
                stack.pop();
                int hei = heigh[tmp];
                int currMax = !stack.empty()?(i - stack.top()+1)*hei:hei*i;
                max_num = max_num < currMax?currMax:max_num;
            }
        }
        return max_num;
    }
};

int main(int argc, char const *argv[])
{

    Solution solution;
    
    
    char a[]= {'1','0','1','0','0'};
    char b[]= {'1','0','1','1','1'};
    char c[]= {'1','1','1','1','1'};
    char d[]= {'1','0','0','1','0'};
    vector<char> arr1(a,a+5);
    vector<char> arr2(b,b+5);
    vector<char> arr3(c,c+5);
    vector<char> arr4(d,d+5);

    vector<vector<char>> tmp(4);
    tmp.push_back(arr1);
    tmp.push_back(arr2);
    tmp.push_back(arr3);
    tmp.push_back(arr4);
    
    solution.maximalRectangle(tmp);
    return 0;
}
