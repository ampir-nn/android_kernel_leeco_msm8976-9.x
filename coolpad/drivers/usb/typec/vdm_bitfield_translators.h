#ifndef __VDM_BITFIELD_TRANSLATORS_H__
#define __VDM_BITFIELD_TRANSLATORS_H__

/*
 * Functions that convert bits into internal header representations...
 */
	UnstructuredVdmHeader 	getUnstructuredVdmHeader(UINT32 in);	// converts 32 bits into an unstructured vdm header struct
	StructuredVdmHeader 	getStructuredVdmHeader(UINT32 in);		// converts 32 bits into a structured vdm header struct
	IdHeader 				getIdHeader(UINT32 in);					// converts 32 bits into an ID Header struct
	VdmType 				getVdmTypeOf(UINT32 in);				// returns structured/unstructured vdm type

/*
 * Functions that convert internal header representations into bits...
 */
	UINT32 	getBitsForUnstructuredVdmHeader(UnstructuredVdmHeader in);	// converts unstructured vdm header struct into 32 bits
	UINT32 	getBitsForStructuredVdmHeader(StructuredVdmHeader in);		// converts structured vdm header struct into 32 bits
	UINT32 	getBitsForIdHeader(IdHeader in);							// converts ID Header struct into 32 bits

/*
 * Functions that convert bits into internal VDO representations...
 */
	CertStatVdo 			getCertStatVdo(UINT32 in);
	ProductVdo 				getProductVdo(UINT32 in);
	CableVdo 				getCableVdo(UINT32 in);
	AmaVdo 					getAmaVdo(UINT32 in);

/*
 * Functions that convert internal VDO representations into bits...
 */
 	UINT32 	getBitsForProductVdo(ProductVdo in);	// converts Product VDO struct into 32 bits
	UINT32 	getBitsForCertStatVdo(CertStatVdo in);	// converts Cert Stat VDO struct into 32 bits
	UINT32	getBitsForCableVdo(CableVdo in);		// converts Cable VDO struct into 32 bits
	UINT32	getBitsForAmaVdo(AmaVdo in);			// converts AMA VDO struct into 32 bits

#endif // header guard

