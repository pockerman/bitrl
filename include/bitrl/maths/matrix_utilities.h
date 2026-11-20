#ifndef MATRIX_UTILITIES_H
#define MATRIX_UTILITIES_H

#include "bitrl/base/bitrl_types.h"

#include <random>
#include <set>
#include <initializer_list>

namespace bitrl{
namespace maths{

///
/// \brief Exchange the rows of the matrix
///
template<typename MatType>
void
exchange_rows(MatType& mat, uint_t r1, uint_t r2){

    for(uint_t col=0; col<mat.columns(); ++col){

        auto tmp = mat(r1, col);
        mat(r1, col) = mat(r2, col);
        mat(r2, col) = tmp;
    }
}

///
/// \brief Shuffle the rows of the matrix.
/// Returns a set with the rows that have been shuffled
///
template<typename MatType>
std::set<uint_t>
shuffle_matrix_rows(MatType& mat){

    uint_t min = 0;
    uint_t max = mat.rows();

    //Will be used to obtain a seed for the random number engine
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint_t> random(min, max);

    std::set<uint_t> touched;

    for(uint_t r=0; r<max; ++r){

        // this row was touched
        if(touched.find(r) != touched.end()){
            continue;
        }

        uint_t candidate = random(gen);

        if(touched.find(candidate) != touched.end()){
            continue;
        }

        exchange_rows(mat, r, candidate);
        touched.insert(r);
        touched.insert(candidate);
    }

    return touched;
}

///
/// \brief Extract the cidx-th column from the matrix
///
/*
template<typename T>
DynVec<T>
get_column(const DynMat<T>& matrix, uint_t cidx){

    DynVec<T> column(matrix.rows(), 0.0);

    for(uint_t r=0; r<matrix.rows(); ++r){
        column[r] =  matrix(r, cidx);
    }
    return column;
}
*/

///
/// \brief get_column_means. Computes the means of the matrix columns
///
/*
template<typename T>
DynVec<real_t>
get_column_means(const DynMat<T>& matrix){

    DynVec<real_t> means(matrix.columns(), 0.0);

    for(uint_t c=0; c<matrix.columns(); ++c){
       auto column =  get_column(matrix, c);
       means[c] = mean(column);
    }

    return means;

}

*/

///
/// \brief get_column_variances. Computes the variances of the matrix columns
///
/*
template<typename T>
DynVec<real_t>
get_column_variances(const DynMat<T>& matrix){

    DynVec<real_t> means(matrix.columns(), 0.0);

    for(uint_t c=0; c<matrix.columns(); ++c){
       auto column =  get_column(matrix, c);
       means[c] = var(column);
    }

    return means;

}
*/

///
/// \brief Extract the row_idx-th row from the matrix
///
template<typename T>
DynVec<T>
get_row(const DynMat<T>& matrix, uint_t row_idx){
    return matrix.row(row_idx);
}

template<typename T>
T
get_row_max(const DynMat<T>& matrix, uint_t row_idx){

    auto row = get_row(matrix, row_idx);
    return row.maxCoeff();
}

template<typename T>
T
get_row_min(const DynMat<T>& matrix, uint_t row_idx){

    auto row = get_row(matrix, row_idx);
    return row.maxCoeff();
}

/*template<typename T>
uint_t
row_argmax(const DynMat<T>& matrix, uint_t row_idx){
    auto row = get_row(matrix, row_idx);
    return blaze::argmax(row);
}


template<typename T>
uint_t
row_argmin(const DynMat<T>& matrix, uint_t row_idx){
    auto row = get_row(matrix, row_idx);
    return blaze::argmin(row);
}


template<typename T>
void 
extract_randomly(const DynMat<T>& matrix, std::vector<DynVec<T>>& rsult,
				 uint_t how_many, bool allow_duplicates=false){

    rsult.clear();
    rsult.reserve(how_many);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, matrix.rows() - 1);
    std::set<uint_t> touched;

    while(rsult.size() != how_many){

        uint_t idx = dis(gen);

        // this row was touched
        if((touched.find(idx) != touched.end()) && !allow_duplicates){
            continue;
        }

        rsult.push_back(get_row(matrix, idx));
        touched.insert(idx);
    }
}

///
/// \brief center_columns. Center the columns of the matrix around
/// the given values
///
template<typename T>
void center_columns(DynMat<T>& matrix, const DynVec<T>& vals){

    for(uint_t r=0; r<matrix.rows(); ++r){
        for (uint_t c=0; c<matrix.columns(); ++c){
            matrix(r, c ) -= vals[c];
        }
    }
}

///
/// \brief create_identity_matrix. Create an identity matrix
/// with shape nrows x nrows.
///
template<typename T>
DynMat<T> create_identity_matrix(uint_t nrows){

    DynMat<T> mat(nrows, nrows);

    for(uint_t r=0; r<nrows; ++r){
        for(uint_t c=0; c<nrows; ++c){
            if(c == r){
                mat(r, r) = static_cast<T>(1);
            }
            else{
               mat(r, c) = static_cast<T>(0);
            }
        }
    }

    return mat;
}

///
/// \brief create_identity_matrix. Create a diagonal matrix
/// with shape diagonal.size()  x diagonal.size().
///
template<typename T>
DynMat<T> create_diagonal_matrix(const std::vector<T>& diagonal){

    DynMat<T> mat(diagonal.size(), diagonal.size());

    for(uint_t r=0; r<diagonal.size(); ++r){
        for(uint_t c=0; c<diagonal.size(); ++c){
            if(c == r){
                mat(r, r) = diagonal[r];
            }
            else{
               mat(r, c) = static_cast<T>(0);
            }
        }
    }

    return mat;
}


///
/// \brief create_identity_matrix. Create an identity matrix
/// with shape nrows x nrows.
///
template<typename T>
DynMat<T> create_diagonal_matrix(const std::initializer_list<T>& diagonal){

    std::vector<T> diag(diagonal.begin(), diagonal.end());
    return create_diagonal_matrix(diag);
}*/


}
}

#endif // MATRIX_SHUFFLER_H
