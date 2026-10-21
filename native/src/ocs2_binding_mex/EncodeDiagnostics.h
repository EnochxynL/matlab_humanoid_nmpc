#pragma once
#include "mex.h"
#include "g1_controller/Diagnostics.h"
#include <algorithm>

inline mxArray* encodeDiagnostics(const g1_controller::DiagnosticBatch& batch) {
  const char* fields[]={"tables","snapshot","configurationJson","loadedLibraries"};
  auto* result=mxCreateStructMatrix(1,1,4,fields);
  const char* tableFields[]={"name","columns","values"};
  auto* tables=mxCreateStructMatrix(1,batch.tables.size(),3,tableFields);
  for(size_t k=0;k<batch.tables.size();++k) {
    const auto& table=batch.tables[k];mxSetField(tables,k,"name",mxCreateString(table.name.c_str()));
    auto* columns=mxCreateCellMatrix(1,table.columns.size());
    for(size_t j=0;j<table.columns.size();++j)mxSetCell(columns,j,mxCreateString(table.columns[j].c_str()));
    mxSetField(tables,k,"columns",columns);
    auto* values=mxCreateCellMatrix(table.rows.size(),table.columns.size());
    for(size_t i=0;i<table.rows.size();++i)for(size_t j=0;j<table.columns.size();++j) {
      const auto& v=table.rows[i][j];
      mxSetCell(values,i+j*table.rows.size(),v.textual?mxCreateString(v.text.c_str()):mxCreateDoubleScalar(v.number));
    }
    mxSetField(tables,k,"values",values);
  }
  mxSetField(result,0,"tables",tables);
  auto* snapshot=mxCreateDoubleMatrix(batch.snapshot.size(),1,mxREAL);
  std::copy(batch.snapshot.begin(),batch.snapshot.end(),mxGetPr(snapshot));
  mxSetField(result,0,"snapshot",snapshot);
  mxSetField(result,0,"configurationJson",mxCreateString(batch.configurationJson.c_str()));
  mxSetField(result,0,"loadedLibraries",mxCreateString(batch.loadedLibraries.c_str()));return result;
}
