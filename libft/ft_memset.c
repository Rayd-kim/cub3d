/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_memset.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/11/17 14:45:05 by youskim           #+#    #+#             */
/*   Updated: 2021/11/29 19:09:16 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

void	*ft_memset(void *b, int c, size_t n)
{
	unsigned char	a;
	unsigned char	*str;
	size_t			i;

	a = (unsigned char)c;
	str = (unsigned char *)b;
	i = 0;
	while (i < n)
	{
		str[i] = a;
		i++;
	}
	return (str);
}
