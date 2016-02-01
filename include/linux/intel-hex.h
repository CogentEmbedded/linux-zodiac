#ifndef _LINUX_INTEL_HEX_H
#define _LINUX_INTEL_HEX_H

/* Parses INTEL HEX formatted line */
int parse_hex_line(unsigned char *in_data, unsigned char *addr,
		unsigned char *out_data, unsigned char *out_length,
		unsigned char *addr_has_changed);

#endif /* _LINUX_INTEL_HEX_H */
