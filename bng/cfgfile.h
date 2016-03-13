/*
  Copyright(c) 2010-2014 Intel Corporation.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _CFG_FILE_H_
#define _CFG_FILE_H_

#include <stdio.h>

#define DEFAULT_CONFIG_FILE	"./dppd.cfg"

/* configuration file line parser procedure */
typedef int (*cfg_parser)(unsigned sindex, char *str, void *data);

#define CFG_INDEXED	0x80000000	/* section contains index [name #] */
#define MAX_INDEX	32

struct cfg_section {
	const char	*name;	/* section name without [] */
	cfg_parser	parser;	/* section parser function */
	void		*data;	/* data to be passed to the parser */
	/* set by parsing procedure */
	unsigned	indexp[MAX_INDEX];
	int		nbindex;
	int		error;
};

#define MAX_CFG_STRING_LEN 512
#define STRING_TERMINATOR_LEN 4

struct cfg_file {
	char		*name;
	FILE		*pfile;
	unsigned	line;
	unsigned	index_line;
	/* set in case of any error */
	unsigned	err_line;
	char		*err_section;
	unsigned	err_entry;
	char		cur_line[MAX_CFG_STRING_LEN + STRING_TERMINATOR_LEN];
};

struct cfg_file *cfg_open(const char *cfg_name);
int cfg_parse(struct cfg_file *pcfg, struct cfg_section *psec);
int cfg_close(struct cfg_file *pcfg);

/* The delimiter for csv files is ',' and spaces before and after the
   delimiter are ignored. Lines starting with # are skipped
   completely. The line is valid only if the number of fields is >=
   min_fields and <= max_fields. These bounds are configurable when
   the file is opened. */

struct csv_file {
	FILE		*fp;
	int              line;
	int              min_fields;
        int              max_fields;
	char            **fields;
	char            *str;
	char            delim;
};

struct csv_file* csv_open(const char *name, int min_fields, int max_fields);
struct csv_file* csv_open_delim(const char *name, int min_fields, int max_fields, char delim);
/* Returns number of fields read in last line or negative on failure */
int csv_next(struct csv_file *file, char** fields);
int csv_close(struct csv_file *file);

#endif /* _CFGFILE_H_ */
