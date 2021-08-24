/*
 *
 * Confidential Information of Telekinesys Research Limited (t/a Havok). Not for disclosure or distribution without Havok's
 * prior written consent. This software contains code, techniques and know-how which is confidential and proprietary to Havok.
 * Product and Trade Secret source code contains trade secrets of Havok. Havok Software (C) Copyright 1999-2013 Telekinesys Research Limited t/a Havok. All Rights Reserved. Use of this software is subject to the terms of an end user license agreement.
 *
 */
#ifndef HK_MATH_MATRIXf_NM_H
#define HK_MATH_MATRIXf_NM_H

class hkVectorNf;

/// A 2d matrix of any size.
/// Data is stored in column order in memory
class hkMatrixfNm
{
	public:
		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_MATH, hkMatrixfNm);

			/// Returns true if the matrix is square
		hkBool isSquare() const { return m_numCols > 0 && m_numCols == m_numRows; }

			/// Set to the identity (only possible on square matrix)
		void setIdentity();

			/// Returns true if is the identity
		hkBool isIdentity(hkFloat32 epsilon = 1e-5f) const;

			/// Set the size
		void setSize(int numRows, int numCols);

			/// Returns if the matrix is ok
		hkBool isOk() const;

			/// Returns true if the matrix is symmetric (must be square, and its transpose equal itself)
		hkBool isSymmetric(hkFloat32 threshold = 1e-5f) const;

			/// Set from a column ordered array
		void set( int numRows, int numCols, const hkFloat32* v);

			/// Assignment
		void operator=(const hkMatrixfNm& rhs) { m_numCols = rhs.m_numCols; m_numRows = rhs.m_numRows; m_elements = rhs.m_elements; }

			/// Set the transpose
		void setTranspose(const hkMatrixfNm& rhs);

			/// Accumulate another matrix. Must be the same size
		void add(const hkMatrixfNm& rhs);
			/// Set to a and b added
		void setAdd(const hkMatrixfNm& a, const hkMatrixfNm& b);

			/// Subtract rhs from this. Must be the same size
		void sub(const hkMatrixfNm& rhs);
			/// Set to a - b
		void setSub(const hkMatrixfNm& a, const hkMatrixfNm& b);

			/// Returns true if they are equal
		hkBool equals(const hkMatrixfNm& rhs, const hkFloat32 threshold = 1e-5f) const;


			/// Multiply all components by v
		void mul(hkSimdFloat32Parameter v);

			/// Get the column
		void getColumn(int col, hkVectorNf& colOut) const;
			/// Get the row
		void getRow(int row, hkVectorNf& rowOut) const;
			/// Get an alias to the actual columnin the matrix - altering the vector will alter the matrix, and the vector
			/// will only be valid whilst the matrix is in scope, and its size does not change
		void getColumnAlias(int row, hkVectorNf& rowOut);

			/// Set as the outer product of a and b
		void setOuterProduct(const hkVectorNf& a, const hkVectorNf& b);

			/// Get the number of rows
		HK_FORCE_INLINE int getNumRows() const { return m_numRows; }
			/// Get the number of columns
		HK_FORCE_INLINE int getNumColumns() const { return m_numCols; }

			/// Get the vector stride between columns
		HK_FORCE_INLINE int getVectorStride() const { return (m_numRows + 3) >> 2; }
			/// Get the float stride between columns
		HK_FORCE_INLINE int getRealStride() const { return getVectorStride() * 4; }

			/// Get the elements as reals. They are stored in column order, of realStride reals between columns
		HK_FORCE_INLINE hkFloat32* getRealElements() { return (hkFloat32*)m_elements.begin(); }
			/// Get the elements as reals. They are stored in column order, of realStride reals between columns
		HK_FORCE_INLINE const hkFloat32* getRealElements() const { return (const hkFloat32*)m_elements.begin(); }

			/// Element access
		hkFloat32& operator() ( int row, int col) { HK_ASSERT(0x23423424, col >= 0 && col < m_numCols && row >= 0 && row < m_numRows); return ((hkFloat32*)m_elements.begin())[col * getRealStride() + row]; }
			/// Element access
		const hkFloat32& operator() (int row, int col) const { HK_ASSERT(0x2423a432, col >= 0 && col < m_numCols && row >= 0 && row < m_numRows); return ((const hkFloat32*)m_elements.begin())[col * getRealStride() + row]; }

			/// Get the vectors
		HK_FORCE_INLINE hkVector4f* getVectors() { return m_elements.begin(); }
		HK_FORCE_INLINE const hkVector4f* getVectors() const { return m_elements.begin(); }

			/// Multiply the matrix with the column vector 'in'. Store in the column vector 'out'.
		void multiply(const hkVectorNf& in, hkVectorNf& out) const;

			/// Invert the matrix in-place using Gaussian elimination. Searches for maximum pivots to
			/// improve accuracy.
			/// If matrix is singular (not invertible) will return HK_FAILURE.
		hkResult invert();

			/// Set to a multiplied by b
		void setMul(const hkMatrixfNm& a, const hkMatrixfNm& b);
			/// Multiply on FPU
		void setFpuMul(const hkMatrixfNm& a, const hkMatrixfNm& b);

			/// Ctor
		hkMatrixfNm(int numRows, int numCols) { _setSize(numRows, numCols); }
			/// Copy ctor
		hkMatrixfNm(const hkMatrixfNm& rhs);
			/// Default Ctor
		hkMatrixfNm() { m_numCols = 0; m_numRows = 0;}
			/// Construct from data values. Values are presented in column order
		hkMatrixfNm(int numRows, int numCols, const hkFloat32* values);

#ifdef HK_DEBUG
		static void HK_CALL selfTest();
#endif

	protected:
		
		static const hkVector4ComparisonMask::Mask s_zeroMask[4];

		void _setSize(int numRows, int numCols);
		static void _setValues( int numRows, int numCols, const hkFloat32* v, hkVector4f* dst);

		// Data is held in column order. Each row is rounded up to 4 elements in size
		hkArray<hkVector4f> m_elements;
		int m_numRows;
		int m_numCols;


};

#endif //HK_MATH_MATRIXf_N_M_H

/*
 * Havok SDK - Base file, BUILD(#20131218)
 * 
 * Confidential Information of Havok.  (C) Copyright 1999-2013
 * Telekinesys Research Limited t/a Havok. All Rights Reserved. The Havok
 * Logo, and the Havok buzzsaw logo are trademarks of Havok.  Title, ownership
 * rights, and intellectual property rights in the Havok software remain in
 * Havok and/or its suppliers.
 * 
 * Use of this software for evaluation purposes is subject to and indicates
 * acceptance of the End User licence Agreement for this product. A copy of
 * the license is included with this software and is also available from salesteam@havok.com.
 * 
 */
