NDIS_STATUS MiniportQueryInformationOuter (
  NDIS_HANDLE Context,
  NDIS_OID Oid,
  PVOID InformationBuffer,
  ULONG InformationBufferLength,
  PULONG BytesWritten,
  PULONG BytesNeeded
);

NDIS_STATUS MiniportSetInformationOuter (
  NDIS_HANDLE Context,
  NDIS_OID Oid,
  PVOID InformationBuffer,
  ULONG OidLength,
  PULONG BytesRead,
  PULONG BytesNeeded
);
