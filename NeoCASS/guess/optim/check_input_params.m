function pdcylin = check_input_params(pdcylin, geo)

  DEF_SPITCH = 0.16;
  DEF_RPITCH = 0.55;
  DEF_NSPAR  = 2;

  
  if isfield(pdcylin, 'spitch')
    
    if (pdcylin.spitch<eps)
      pdcylin.spitch = DEF_SPITCH;
    end
  else
    pdcylin.spitch = DEF_SPITCH;
    fprintf('\n\t### Warning: stringer pitch not defined in material_property.wing.spitch');
  end

   
  if isfield (pdcylin,'rpitch')
    if (pdcylin.rpitch<eps)
      pdcylin.rpitch = DEF_RPITCH;
    end
  else
    pdcylin.rpitch = DEF_RPITCH;
    fprintf('\n\t### Warning: rib pitch not defined in material_property.wing.rpitch');
  end

  
  if isfield(pdcylin , 'nspar')
    if (pdcylin.nspar<eps)
      pdcylin.nspar = DEF_NSPAR;
    end
  else
    pdcylin.nspar = DEF_NSPAR;
    fprintf('\n\t### Warning: number of spars not defined in material_property.wing.nspar');
  end
  %           check data
  if (geo.y(end)<pdcylin.rpitch)
    fprintf('\n\t### Warning: rib pitch exceeds wing span.');
  end

end