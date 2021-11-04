#!/usr/bin/perl
open F,qq(LC_ALL=C readelf -e u-boot|sed 's/\\[ /\\[/'|);
$addr=@ARGV?hex($ARGV[0]):0x8ff3c000;
$textbase=0x80100000;
#$textbase=0xbfc00000;
while (<F>)
{
 if(/\[\s*\d+\] [.]/){
  @F = split('\s+',$_);
  next if(!hex($F[4]));
  $textbase = hex($F[4]) if ($F[2] eq ".text");
  $s{$F[2]}=sprintf qq(0x%x),hex($F[4])-$textbase+$addr; 
  $a = $a . sprintf( qq( -s %s 0x%x), $F[2],hex($F[4])-$textbase+$addr);
  } 
}
print qq(add-symbol-file ) , $ENV{qq(PWD)} , qq(/u-boot  $s{q(.text)} ) , $a, qq(\n);
