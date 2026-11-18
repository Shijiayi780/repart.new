
#ifdef REPART_DETECTOR_TYPES_ONLY
long long Hash(int x);
#else
long long Hash(int x) {
  return x * x;
}
#endif
